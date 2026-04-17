# truck-bop mission notes

## Current mission target

This mission is intentionally limited to making `truck-bop` basic booleans behave correctly for **real periodic sphere + box** while preserving key `box-box` regression sentinels.

### Required outcome
- keep public API unchanged
- satisfy legality-first gates for sphere-box:
  - command succeeds
  - exact solid count matches expectation
  - returned shells are closed
- preserve named box-box sentinels in `bool_occt_verify`

## Current root-cause zones

### Periodic seam classification
- `truck-bop/src/fclass2d.rs` is the explicit periodic-domain classifier
- seam-wrap fixes must preserve `Inside` / `OnBoundary` semantics across one-period shifts

### FF projection / section root
- `truck-bop/src/intersect/ff.rs` owns the curved-surface section path that sphere-box depends on
- sphere-box `section` is a required smoke gate, not optional debug noise

### Trim / dedup / assembly
- `truck-bop/src/trim.rs` is where seam-equivalent points, duplicate fragments, shared-edge sewing, shell grouping, and solid gates must converge
- do not hide trim/assembly defects behind fallback behavior higher up the stack

## Hard invariants

- `assemble_shells` must reject non-closed / non-orientable shell assemblies
- `build_solids_from_shells` must reject non-closed shell input
- `fuse` fallback to multi-solid passthrough is mission-failing for overlap/tangent cases that should return one closed solid
- sphere/box fixtures stay internal to tests/examples only

## Box-box sentinel rows to preserve

- `overlap_common`
- `overlap_fuse`
- `overlap_cut`
- `edge_touch_fuse`
- `adjacent_fuse`
- `offset_fuse`
- `offset_cut`
- `disjoint_fuse`

## Execution order

1. restore baseline seam and invariant validators
2. add sphere-box rows and targeted validators
3. make overlap / contained cases pass legality-first gates
4. make tangent / section-root cases pass legality-first gates
5. stabilize seam/dedup/touching logic until one matrix run keeps the required box-box rows green
