# Truck BOP Shell Assembly Repair Implementation Plan

> **Superseded:** 本计划已被 `docs/plans/2026-03-28-truck-bop-trim-pipeline-repair-v2.md` 替代，请以后者为准；不要继续按本文件落地实现。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore `truck-bop` shell assembly so it groups fragments by real shared topology, rejects open shells, rejects globally inverted closed shells, and turns the five failing `trim.rs` shell tests green.

**Architecture:** Stop inferring shared source boundaries from UV polyline shape alone. Carry original boundary edge identity through `TrimmingEdge`, canonicalize adjacency with the priority `section_curve -> original_edge -> true non-section shared trim edge -> open boundary`, then run unconditional closed-shell validation plus a post-assembly global orientation check inside `assemble_shells()`. Keep the change localized to `truck-bop` trimming/shell assembly code and prove it with focused `trim.rs` tests before the full crate regression.

**Tech Stack:** Rust 2021, Cargo, `truck-topology`, `truck-modeling`, existing `truck-bop` unit tests

---

## File Map

- Modify: `truck-bop/src/bopds/interference.rs`
  Add a source-boundary identity field to `TrimmingEdge` and document when it is `Some` vs `None`.
- Modify: `truck-bop/src/trim.rs`
  Capture source edge ids in `boundary_loops()`, update canonical edge selection, tighten `assemble_shells()` validation, and add or adjust the shell tests.
- Modify: `truck-bop/src/bopds/mod.rs`
  Update any DS fixtures or unit tests that construct `TrimmingEdge` directly.
- Modify: `truck-bop/src/intersect/ee.rs`
  Update direct `TrimmingEdge { ... }` construction to initialize the new field with `None`.
- Modify: `truck-bop/src/intersect/ef.rs`
  Update direct `TrimmingEdge { ... }` construction to initialize the new field with `None`.
- Modify: `truck-bop/src/intersect/ve.rs`
  Update direct `TrimmingEdge { ... }` construction to initialize the new field with `None`.

### Task 1: Preserve Source Edge Identity In Trimming Data

**Files:**
- Modify: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/trim.rs`
- Modify: `truck-bop/src/bopds/mod.rs`
- Modify: `truck-bop/src/intersect/ee.rs`
- Modify: `truck-bop/src/intersect/ef.rs`
- Modify: `truck-bop/src/intersect/ve.rs`
- Test: `truck-bop/src/trim.rs`

- [ ] **Step 1: Write the failing test**

Add a unit test in `truck-bop/src/trim.rs` named `boundary_loops_preserve_original_edge_ids_for_source_faces`. Build two adjacent faces from a cuboid, call `boundary_loops()` for both, and assert that the matching shared boundary segment on each face carries the same non-`None` `original_edge` value while non-shared outer edges keep distinct ids.

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop boundary_loops_preserve_original_edge_ids_for_source_faces -- --exact`
Expected: FAIL because `TrimmingEdge` currently has no source-edge field and `boundary_loops()` discards boundary identity.

- [ ] **Step 3: Write minimal implementation**

Add `original_edge: Option<EdgeId>` to `TrimmingEdge`. In `boundary_loops()`, iterate each wire's source edges instead of rebuilding the loop only from `windows(2)`, so every source-face boundary segment stores the real topological edge id. Set `original_edge: None` for section-curve-generated edges and all synthetic test helpers that do not come from a real source edge. If `truck_topology::Edge::id()` needs conversion, add one private helper in `trim.rs` instead of scattering casts.

- [ ] **Step 4: Run test to verify it passes**

Run:
`cargo test -p truck-bop boundary_loops_preserve_original_edge_ids_for_source_faces -- --exact`
`cargo check -p truck-bop`
Expected: the new test passes and the crate compiles after every `TrimmingEdge { ... }` call site initializes `original_edge`.

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/bopds/interference.rs truck-bop/src/trim.rs truck-bop/src/bopds/mod.rs truck-bop/src/intersect/ee.rs truck-bop/src/intersect/ef.rs truck-bop/src/intersect/ve.rs
git commit -m "feat: preserve source edge identity in trimming data"
```

### Task 2: Rebuild Component Adjacency On Real Shared Topology

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [ ] **Step 1: Write the failing test**

Use the existing red tests as the contract:
- `shell_assembly_groups_faces_by_shared_topology_components`
- `face_orientation_flips_adjacent_face_to_close_shell`
- `shell_assembly_separates_disconnected_closed_components`

- [ ] **Step 2: Run test to verify it fails**

Run:
`cargo test -p truck-bop shell_assembly_groups_faces_by_shared_topology_components -- --exact`
`cargo test -p truck-bop face_orientation_flips_adjacent_face_to_close_shell -- --exact`
`cargo test -p truck-bop shell_assembly_separates_disconnected_closed_components -- --exact`
Expected: FAIL with broken component grouping or wrong shell counts.

- [ ] **Step 3: Write minimal implementation**

Update `canonical_rebuilt_edge()` so the precedence is:
1. `section_curve` -> `CanonicalRebuiltEdge::Source(...)`
2. `original_edge` -> `CanonicalRebuiltEdge::Source(...)`
3. verified non-section shared trim edge -> `CanonicalRebuiltEdge::SharedBoundary(...)`
4. everything else -> `CanonicalRebuiltEdge::OpenBoundary(...)`

Keep `edge_is_shared_non_section()` strict: it should continue to identify only true split-generated non-section shared trim edges, not ordinary two-point source boundaries. Update any artificial shared-trim-edge tests so they still cover this fallback path.

- [ ] **Step 4: Run test to verify it passes**

Run:
`cargo test -p truck-bop shell_assembly_groups_faces_by_shared_topology_components -- --exact`
`cargo test -p truck-bop face_orientation_flips_adjacent_face_to_close_shell -- --exact`
`cargo test -p truck-bop shell_assembly_separates_disconnected_closed_components -- --exact`
`cargo test -p truck-bop canonical_rebuilt_edge_ids_distinguish_shared_and_open_boundaries -- --exact`
`cargo test -p truck-bop canonical_rebuilt_edge_ids_share_true_non_section_trim_edges_only -- --exact`
Expected: the three shell-grouping tests pass, the open-boundary distinction test still passes, and the synthetic shared-trim-edge fallback still works.

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs
git commit -m "fix: restore shell adjacency from shared topology"
```

### Task 3: Make `assemble_shells()` Reject Every Non-Closed Shell

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [ ] **Step 1: Write the failing test**

Use the existing test `shell_closure_rejects_boundary_edge_component` as the red test. Add a second assertion in the same test module only if needed to pin the exact error path after refactoring.

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop shell_closure_rejects_boundary_edge_component -- --exact`
Expected: FAIL because `assemble_shells()` currently gates the closure check behind `shell_faces_reused_original_boundaries(...)` and `shell.len() > 1`.

- [ ] **Step 3: Write minimal implementation**

Remove `shell_faces_reused_original_boundaries()` from the decision path. After each shell is assembled, immediately return `BopError::TopologyInvariantBroken` when `shell.shell_condition() != ShellCondition::Closed`. If the helper becomes unused, delete it in this task rather than leaving dead code behind. Keep the validation in `assemble_shells()` so callers never receive open shells.

- [ ] **Step 4: Run test to verify it passes**

Run:
`cargo test -p truck-bop shell_closure_rejects_boundary_edge_component -- --exact`
`cargo test -p truck-bop shell_assembly_separates_disconnected_closed_components -- --exact`
Expected: the open-shell case now returns `TopologyInvariantBroken` and the positive disconnected-shell case still passes.

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs
git commit -m "fix: reject non-closed shells during assembly"
```

### Task 4: Add Post-Assembly Global Orientation Validation

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [ ] **Step 1: Write the failing test**

Use the existing test `shell_orientation_rejects_closed_shell_with_inverted_faces` as the red test. Pair it with the already-green target `face_orientation_flips_adjacent_face_to_close_shell` so the task preserves local reorientation while rejecting a globally inverted shell.

- [ ] **Step 2: Run test to verify it fails**

Run:
`cargo test -p truck-bop shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact`
`cargo test -p truck-bop face_orientation_flips_adjacent_face_to_close_shell -- --exact`
Expected: the inverted-shell test fails while the positive orientation test documents the intended success case.

- [ ] **Step 3: Write minimal implementation**

Add a private `validate_shell_orientation()` helper in `truck-bop/src/trim.rs` and call it from `assemble_shells()` immediately after the closed-shell check. Implement it as a global check, not another local edge comparison:
1. Build a temporary `Solid::new(vec![shell.clone()])`.
2. Pick one face from the shell and derive a stable sample point from its boundary vertices.
3. Compute or approximate that face's oriented normal from the face surface and orientation.
4. Offset a small distance along `+normal` and `-normal`.
5. Use the existing point-classification utilities to verify `+normal` lands outside the solid and `-normal` lands inside.
6. Return `TopologyInvariantBroken` when the relation is reversed or ambiguous within tolerance.

If `truck-topology` already exposes a more robust outward-orientation check, prefer it over custom sampling, but keep the validation inside `assemble_shells()` because the failing test expects rejection there.

- [ ] **Step 4: Run test to verify it passes**

Run:
`cargo test -p truck-bop shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact`
`cargo test -p truck-bop face_orientation_flips_adjacent_face_to_close_shell -- --exact`
`cargo test -p truck-bop shell_assembly_separates_disconnected_closed_components -- --exact`
Expected: the globally inverted shell is rejected, the locally fixable shell still assembles, and the disconnected closed-shell case stays green.

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs
git commit -m "fix: validate assembled shell orientation"
```

### Task 5: Run Full Regression And Remove Residual Dead Code

**Files:**
- Modify: `truck-bop/src/trim.rs` (only if cleanup remains after Tasks 1-4)

- [ ] **Step 1: Write the failing test**

No new test code. Use the crate-wide test suite as the regression contract.

- [ ] **Step 2: Run test to verify current state**

Run: `cargo test -p truck-bop`
Expected: if any failures remain, they should be real regressions from the shell-assembly work rather than the original five known reds.

- [ ] **Step 3: Write minimal implementation**

Remove leftover dead helpers, duplicated conversions, or temporary diagnostics introduced during Tasks 1-4. Do not broaden scope beyond shell-assembly correctness and constructor cleanup.

- [ ] **Step 4: Run test to verify it passes**

Run:
`cargo test -p truck-bop`
`cargo check -p truck-bop`
Expected: the full `truck-bop` test suite passes and the crate builds cleanly.

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs truck-bop/src/bopds/interference.rs truck-bop/src/bopds/mod.rs truck-bop/src/intersect/ee.rs truck-bop/src/intersect/ef.rs truck-bop/src/intersect/ve.rs
git commit -m "fix: restore truck-bop shell assembly invariants"
```

## Acceptance Checklist

- [ ] `boundary_loops()` preserves real source-edge identity for source-face boundaries.
- [ ] `shell_assembly_groups_faces_by_shared_topology_components` passes without turning ordinary open boundaries into shared boundaries.
- [ ] `face_orientation_flips_adjacent_face_to_close_shell` and `shell_assembly_separates_disconnected_closed_components` pass.
- [ ] `shell_closure_rejects_boundary_edge_component` passes because `assemble_shells()` rejects all non-closed shells.
- [ ] `shell_orientation_rejects_closed_shell_with_inverted_faces` passes because `assemble_shells()` performs a global orientation check.
- [ ] `cargo test -p truck-bop` passes cleanly.
