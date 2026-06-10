# Static Prefix Substitution for Key Compression — Implementation Plan

## Motivation

On prefix-heavy workloads (e.g. Meesho `catalog__user_geohash_1_3:<id>|<n>`, 40 chars with a
constant 26-char prefix) the per-key RAM floor is dominated by:

- `type_used_memory_key` — the key-string heap allocation (~48 B/key; ~456 GB at 9.5B keys).
- `table_used_memory` — the DashTable slot cost (~73 B/key; ~694 GB), scales with entry count.

Keys land in a separate heap allocation only because they exceed `CompactObj::kInlineLen = 16`
(`compact_object.h:152`). Huffman (the only existing key transform) is order-0 entropy coding,
measured ratio **0.585** on these keys: 40 → ~23 bytes, still > 16, so the key stays heap-allocated.
It cannot exploit the cross-key repeated prefix.

**Static prefix substitution** replaces a configured long prefix with a short internal token:
`catalog__user_geohash_1_3:232472614|4442` → `<token>232472614|4442` ≈ 15 bytes → **inline**,
eliminating the ~456 GB key heap with **no change to slot size** (`table_used_memory` stays 694 GB).
Net tiered effect at 9.5B keys: `used_memory` ~1.14 TB → ~760 GB (1.5 TB box → 1 TB).

## Configuration (static, flag-driven — no learned dictionary)

A startup flag supplies the prefix list directly; no training command, no dictionary persistence:

```
--key_prefix_substitution="catalog__user_geohash_1_3:|g;orders:|o"
--enable_key_prefix_substitution=true        # or: non-empty flag implies enabled (huffman style)
```

- Each entry is `<prefix>|<token>`; entries separated by `;`. `|` is a safe within-entry separator
  because a configured prefix never contains `|` (it ends at `:`); `;` is safe because prefixes
  contain `:`, not `;`. (The key *suffix* may contain `|`, but suffixes never appear in this flag.)
- **The token is exactly ONE byte**, user-chosen and unique across entries (up to 255 prefixes).
  Fixed 1-byte width makes decode unambiguous: read the `PREFIX_ENC` tag, read one token byte, map
  to its prefix, prepend.
- **Inline guarantee + fallback:** stored form is `[token][suffix]` = 1 + suffix_len. With the
  variable part ≤ 14 chars → 15 bytes ≤ `kInlineLen` (16) → inline. Validate at startup that
  `1 + expected_max_suffix ≤ 16`. If a particular key's suffix pushes the encoded form > 16 at
  runtime, substitution STILL applies (smaller heap allocation) but that key is not inline —
  substitution is always a net win, inline is the bonus when it fits. Reject tokens longer than 1
  byte at flag-parse to preserve the headroom.
- **Immutability is free:** the list is fixed at startup, so prefix→token is stable by construction.
  Adding a prefix = append a new `<prefix>|<token>` entry + restart. Removing/reordering or changing
  a token corrupts existing keys — refuse to start if the list shrinks/reorders/retokenizes against a
  loaded RDB, or document loudly.

## Critical: collision safety

A pure find-replace with a **printable** token is unsafe:

```
catalog__user_geohash_1_3:232472614  → "g232472614"   (substituted)
"g232472614"  (a literal key)         → "g232472614"   ← aliases the above → corruption
```

Two ways to make it safe; this choice drives the architecture:

### Approach B (recommended): internal `PREFIX_ENC` encoding tag

The substituted form carries a tag bit on the `CompactObj`, so tagged `[id]232472614` is a different
object from a literal `g232472614` under **tag-aware hash/equality**. A printable/compact 1-byte id
is then safe. Decode is centralized (one site materializes the key), and it composes with huffman
(`PREFIX_HUFFMAN_ENC`). Cost: the bitfield surgery below.

### Approach A: reserved-marker rewrite at the key boundary (no CompactObj change)

Use a token that is a **reserved non-printable byte** (e.g. `0x01`) guaranteed absent from real keys.
Rewrite `catalog…:` → `\x01<id>` on ingress; the result is an ordinary ≤16-byte string that goes
inline automatically — **zero changes to `CompactObj`/DashTable**. Tradeoffs:

- Must intercept **every** key ingress (all command key args, replication, RDB load) and **every**
  egress (KEYS, SCAN, RANDOMKEY, keyspace notifications, MONITOR, CLIENT, replication journal, RDB
  save). Missing one egress site leaks `\x01…` to clients; missing an ingress normalization causes a
  silent miss.
- Must **reject (or escape)** any literal incoming key that starts with the reserved byte.

Approach A looks simpler (no core surgery) but the ingress/egress surface is wide and unforgiving.
Approach B localizes correctness in `CompactObj` and is the recommended path; Approach A is viable
for a quick prototype if the egress surface is acceptable and keys are guaranteed printable.

The rest of this plan details **Approach B**.

## The layout constraint: `encoding_` is a full 2-bit field

`compact_object.h:651-654`, packed into the final byte; `sizeof(CompactObj) == 18`
(`compact_object.cc:612`):

```cpp
const bool is_key_ : 1;
uint8_t taglen_   : 5;   // inline length (0..16) OR type tag — needs all 5 bits
uint8_t encoding_ : 2;   // NONE=0 ASCII1=1 ASCII2=2 HUFFMAN=3 — FULL, no free value
```

The `mask_` byte has `uint8_t unused : 2` (`compact_object.h:633`). **Plan: relocate `is_key_` into a
spare `mask_` bit and widen `encoding_` to 3 bits** → room for `PREFIX_ENC=4` (and
`PREFIX_HUFFMAN_ENC=5`), `sizeof` stays 18. This bitfield surgery is step 1 and must keep the 18-byte
static_assert green; every `is_key_`/`encoding_` accessor is re-pointed.

Stored form for `PREFIX_ENC`: `[1-byte id][suffix]`. A 14-char suffix → 15 bytes → fits the existing
16-byte inline buffer. The slot never grows — that is what keeps `table_used_memory` flat.

## Correctness invariant

**Encoding is a deterministic, injective function of the logical key, applied at EVERY key
CompactObj construction — insert AND lookup-probe.**

- Deterministic + fixed flag list ⇒ same logical key → same encoded bytes → same hash + comparison;
  no decode on the GET path (the property huffman already relies on).
- Injective (`[id][suffix]` uniquely reconstructs the key; the tag distinguishes encodings) ⇒ a
  substituted key never aliases a literal one.
- **Audit requirement:** every site that builds a key CompactObj for a lookup must apply
  substitution. Any raw probe for a key that *would* be substituted will miss. Auditing the
  db_slice / find paths is the highest-risk part of the change.

## RDB / replication

Serialize **decoded (logical)** keys in RDB and on the replication wire; re-encode on load using the
receiver's own flag. Keeps snapshots portable across nodes with different/absent config (snapshot
size is not reduced — correctness/portability first). Verify a replica with and without the flag.

## Phased implementation

**Phase 0 — Validate (no engine changes)**
- Offline: take ~1M sampled keys (`RANDOMKEY` dump), apply the configured prefix substitution,
  confirm the fraction that fall ≤16 bytes (expected ~100% for the catalog keys). Confirms the
  inline win before any code.
- Optionally measure the existing huffman delta on the replica to baseline `type_used_memory_key`.

**Phase 1 — Core encoding (Approach B)**
- Bitfield surgery: relocate `is_key_`, widen `encoding_` to 3 bits, add `PREFIX_ENC`. Keep
  `sizeof(CompactObj)==18`.
- Flag parse + thread-local prefix list (mirror `SetHuffmanTable`, `main_service.cc:1099`); init on
  all shards via `RunBriefInParallel` (mirror `InitHuffmanThreadLocal`).
- Encode in `CompactObj::SetString` (`compact_object.cc:~1860`): longest-prefix match → store
  `[id][suffix]` with `PREFIX_ENC`. Decode in `GetString`/`DecodedLen`: prepend prefix list[id].
- Tests: round-trip matched/unmatched/empty-suffix/overlapping-prefix; `sizeof` unchanged; inline
  rate.

**Phase 2 — Hash/compare correctness**
- Enforce the uniform-encoding invariant; audit all key-construction sites (insert + lookup).
- Extend `operator==`/`Hash` for `PREFIX_ENC` (tag-aware).
- Tests: GET/DEL/EXISTS/SCAN/TYPE on substituted keys; mixed matched/unmatched keyspace; explicit
  collision test (literal key equal to a substituted form must not alias).

**Phase 3 — Persistence + ops**
- `--key_prefix_substitution` + enable flag parse at startup.
- RDB/replication decoded-key serialization; replica tests with/without flag.
- Startup guard: refuse a list that shrinks/reorders relative to a loaded dataset.

**Phase 4 — Benchmarks + safety**
- GET/SET throughput + P99 with substitution on vs off (per-key longest-match cost on the hot path).
- Confirm `type_used_memory_key` collapse and `table_used_memory` unchanged.
- ASAN/UBSAN; full `ctest -V -L DFLY`.

**Phase 5 (optional) — `PREFIX_HUFFMAN_ENC`:** strip prefix then huffman the suffix (~14 digits ×
0.5 ≈ 7 bytes) for extra inline headroom on longer suffixes.

## Files to touch

- `src/core/compact_object.h` — bitfield surgery, `EncodingEnum`, accessors.
- `src/core/compact_object.cc` — encode/decode in `SetString`/`GetString`/`DecodedLen`,
  `operator==`/`Hash`; keep `sizeof==18`.
- `src/server/main_service.cc` — flag parse + `SetKeyPrefixTable` (mirror `:740`, `:1099`); init per
  shard.
- `src/server/rdb_save.cc` / `rdb_load.cc` — decoded-key serialization.
- Tests: `src/core/compact_object_test.cc`, `src/server/*_test.cc`.

(No `DEBUG`/training command — the prefix list is static config. This is the main simplification vs a
learned dictionary.)

## Decisions to lock before coding

1. Approach B (tag) vs A (reserved-byte boundary rewrite). B recommended.
2. Bitfield surgery to widen `encoding_` to 3 bits (relocate `is_key_`); confirm nothing depends on
   exact bit positions.
3. `sizeof(CompactObj)` MUST remain 18 — id+suffix live in the existing inline buffer or heap; the
   struct never grows (keeps `table_used_memory` flat).
4. Uniform-encoding invariant + full audit of key-construction/lookup sites.
5. Startup-flag immutability guard (no shrink/reorder against existing data).
6. RDB decoded-key serialization for portability.

## Expected payoff (live analysis, 9.5B-key tiered node)

- `type_used_memory_key` ~456 GB → ~0 (keys inline at ~15 B).
- `table_used_memory` unchanged (~694 GB) — slot size fixed at 18 B.
- `used_memory` ~1.14 TB → ~760 GB; machine tier 1.5 TB → **1 TB**.
- Achieves the inline win huffman structurally cannot, with no application key-format migration and
  no learned-dictionary machinery.
