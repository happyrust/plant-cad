# truck-bop Changelog

## 2026-04-13 — Trim Layer Provenance & Segment Granularity

### Breaking Changes

- **`TrimmingEdge` struct**: `section_curve: Option<SectionCurveId>` + `original_edge: Option<EdgeId>` replaced by single `source: TrimmingEdgeSource` enum.
- **`CanonicalRebuiltEdge` enum**: `Source(EdgeId)` split into `OriginalEdge(EdgeId)` + `SectionSegment(SectionCurveId, VertexId, VertexId)`.

### New Types

- `TrimmingEdgeSource` — discriminated union for edge provenance:
  - `OriginalBoundaryEdge(EdgeId)` — from input solid boundary
  - `SectionCurve(SectionCurveId)` — from face-face intersection
  - `Unattributed` — synthesized during loop repair

### Improvements

- **Closed section segment granularity**: Closed and fallback section loops now generate per-segment `TrimmingEdge` records via `section_polyline_to_segment_edges()`, matching boundary loop granularity.
- **Section segment identity**: `CanonicalRebuiltEdge::SectionSegment` uses `(SectionCurveId, VertexId, VertexId)` so different segments of the same section curve are distinguishable within a face.
- **Cross-face sharing**: `canonical_edges_share_identity()` matches section segments by `SectionCurveId` across faces (vertex IDs differ per-face UV space).
- **Provenance in `rebuild_wire`**: Edge provenance is now extracted via pattern match on `TrimmingEdgeSource` instead of fallible `.get()` + double-Option chain.
- **`collect_sewn_edges`**: Merged redundant if/else branches with identical logic.

### Bug Fixes

- **Fuse selection test**: Fixed pre-existing test that expected `OnBoundary` fragments to be selected for `BooleanOp::Fuse`, contradicting the `Outside`-only implementation.

### Resolved Issues (from ISSUES.md)

- Issue 3 (`classify_point_in_solid` box-only) was already resolved: three-tier classifier (AABB → ray-casting → nearest-face) with passing rotated-box tests.
- Issue 4 (`vertex_ids_for_polyline` collision risk) was already resolved: shared monotonic counter via `bopds.next_generated_vertex_id`.
