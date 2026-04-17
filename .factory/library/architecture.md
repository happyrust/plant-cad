# Architecture

Architectural decisions and behavioral invariants for the `truck-bop` real sphere + box boolean mission.

**What belongs here:** pipeline stages, system invariants, OCCT mapping, and the specific hotspots workers must preserve while implementing sphere/box support and box/box regressions.

---

## Mission Context

This mission does not change the public API. The externally visible entry points remain:

- `truck_bop::common`
- `truck_bop::fuse`
- `truck_bop::cut`
- `truck_bop::section`

The mission goal is to make those paths behave correctly for **real periodic sphere + box** fixtures while preserving key `box-box` sentinel cases. The first acceptance gate is **legality + closed-shell + correct solid count**; OCCT topology-count parity is tightened only after that gate is stable.

## Boolean Pipeline Overview

All three boolean ops flow through the same high-level path in `truck-bop/src/lib.rs`:

1. register source topology into `BopDs`
2. run broad-phase candidate discovery
3. collect low-dimensional interferences (`VV`, `VE`, `VF`, `EE`, `EF`)
4. run face-face intersection in `intersect/ff.rs`
5. rebuild paves / edge-split information in `BopDs`
6. build trimming loops in `trim.rs`
7. derive split faces and classify them against the opposite solid
8. select fragments according to the boolean op
9. merge equivalent vertices and sew fragment edges
10. rebuild faces/shells and build final solids

`section` is a separate public entry point, but it shares the same **FF intersection root** and therefore acts as an early diagnostic for seam and curved-surface failures.

## Mission-Critical Hotspots

### 1. Periodic seam canonicalization

Sphere support is blocked less by public API shape and more by periodic-surface bookkeeping.

- `fclass2d.rs` is the current explicit periodic-domain classifier
- `intersect/ff.rs` projects section samples into face parameter space
- seam-crossing curves must remain topologically continuous after projection

Workers should assume that seam fixes must preserve classifier equivalence across periodic wraps and must not create duplicate vertices/edges downstream.

### 2. Shell / solid invariant gates

The hard correctness gate is not “did an example print something plausible”; it is whether downstream topology is valid:

- `assemble_shells(...)` must reject non-closed / non-orientable shell assemblies
- `build_solids_from_shells(...)` must reject non-closed shell input
- all accepted boolean outputs must be backed by closed shells

This mission relies on these gates remaining strict. Do not weaken them to make examples pass.

### 3. Dedup / sewing / rebuild

The common root cause for sphere tangent bugs and box touching regressions is duplicate or unstable fragment topology:

- seam-equivalent UV points
- duplicated section fragments
- shared-edge reconstruction under touching/adjacent cases
- fallback to multiple passthrough solids in `fuse` when assembly fails

Workers should treat dedup, shared-edge sewing, and shell grouping as one system, not as isolated local fixes.

### 4. Touching / tangent semantics

Touching/tangent behavior is part of the main acceptance gate for this mission.

The important invariants are:

- tangent / touching cases must stay finite and deterministic
- `fuse` must not silently degrade to multi-solid passthrough where one closed merged solid is required
- `cut` must not emit malformed topology in zero-volume-contact cases
- fixing tangent sphere-box behavior must not regress box-box `edge_touch_fuse`, `adjacent_fuse`, or `offset_cut`

## Worker Ownership Map

- `fclass2d.rs`
  - owns periodic seam normalization and classifier equivalence across wraps
  - should be the first place to add seam-specific targeted tests
- `intersect/ff.rs`
  - owns sphere/box FF projection behavior, section-curve parameterization, and section-root continuity
  - should absorb seam-aware projection/orientation fixes, not `trim.rs`
- `trim.rs`
  - owns trimming loops, fragment dedup, merged-vertex clustering, sewing, shell grouping, and solid rebuild gates
  - should absorb shared-edge dedup and touching/tangent assembly fixes
- `lib.rs`
  - owns public boolean orchestration and any explicit passthrough/fallback branching
  - should not be used to hide topology defects that belong in FF/trim/rebuild
- `examples/bool_occt_verify.rs`
  - owns named matrix rows and regression visibility
- targeted/integration tests under `truck-bop`
  - own legality, closed-shell, solid-count, and section-smoke assertions

## Data Model Notes

`truck-bop` follows an OCCT-inspired staged DS model:

- **BopDs**: central indexed data structure
- **Paves / PaveBlocks**: edge-splitting and parameter-space bookkeeping
- **SectionCurve**: FF intersection result consumed by trimming
- **SplitFace**: fragment-level unit used for selection and rebuild
- **MergedVertex / SewnPath**: dedup and sewing intermediates before shell assembly

Important invariant:

- downstream shell reconstruction must operate on **fragment-level trimming data**, not only on `SplitFace.original_face`, because multiple selected fragments can originate from one source face after splitting

## Validation Surfaces

This mission uses two CLI-level validation surfaces:

1. **targeted/integration Cargo tests**
   - authoritative for legality, closed-shell, and exact solid-count assertions on sphere-box cases
2. **`bool_occt_verify` example matrix**
   - authoritative for named matrix rows and box-box regression sentinels

The example matrix is not enough by itself to prove closed-shell status unless the validator explicitly checks it. Workers should therefore pair matrix rows with targeted tests when a feature claims closed-shell assertions.

### Required validators and sentinels

Workers should treat the validation contract as authoritative, but the minimum recurring names to keep in mind are:

- sphere-box targeted validators for overlap / contained / tangent
- sphere-box section smoke validator
- example-matrix sphere-box rows for overlap / contained / tangent
- box-box sentinels in `bool_occt_verify`:
  - `overlap_common`
  - `overlap_fuse`
  - `overlap_cut`
  - `edge_touch_fuse`
  - `adjacent_fuse`
  - `offset_fuse`
  - `offset_cut`
  - `disjoint_fuse`

## Section Role in This Mission

`section` is not optional noise. It is a required smoke gate for the shared FF root path.

- if sphere-box `section` breaks, treat that as mission-relevant, not as a separate nice-to-have
- section validation does not replace boolean validation, but it is the earliest curved-surface diagnostic workers should preserve

## Fallback Rule

`fuse` returning passthrough multi-solid output is acceptable only where the contract explicitly allows that behavior.

For this mission:

- overlap and tangent sphere-box union cases requiring one closed merged solid must treat passthrough multi-solid fallback as a failure
- workers must fix the underlying FF / trim / assembly issue instead of relying on fallback to keep commands green
- do not weaken shell/solid gates to avoid fallback

## Execution Priority Order

Workers should execute in this order:

1. restore legality / closed-shell / solid-count gates
2. add and stabilize sphere-box validators and example rows
3. preserve named box-box sentinels while landing seam/dedup fixes
4. only after the above is stable, tighten toward OCCT topology parity

## Worker Guidance Inferred from the Architecture

- Prefer adding or tightening narrow validators first, then implement the smallest code slice that makes them pass.
- Preserve the current public API and provenance threading.
- Do not “fix” regressions by weakening topology gates or hiding failures behind fallback behavior.
- When adding sphere-box fixtures, keep them internal to tests/examples; do not expose new public constructors or helper APIs.
