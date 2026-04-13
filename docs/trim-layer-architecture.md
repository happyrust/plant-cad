# Truck-bop Trim Layer Architecture

## Overview

The trim layer sits between face-face intersection detection and shell assembly. It transforms raw section curves into structured topology fragments that can be assembled into closed shells.

```
Intersection Detection → [Trim Layer] → Shell Assembly → Solid Construction
    SectionCurve            TrimmingLoop     Shell<C,S>     Solid<C,S>
                            SplitFace
                            SewnEdge
```

## Data Flow

### 1. Loop Construction (`build_trimming_loops`)

**Input**: `(FaceId, Face)` pairs + section curves from `BopDs`

**Process**:
- For faces without sections: extract boundary loops directly
- For faces with open sections: build loops via edge graph (`build_loops_via_edge_graph`)
- For faces with closed sections: split section polyline into per-segment edges
- Classify loops as outer/inner via signed area containment tree

**Output**: `TrimmingLoop` records stored in `BopDs`

### 2. Split Face Construction (`build_split_faces`)

**Input**: Stored trimming loops

**Process**: Group outer loops with their contained inner loops per face. Each outer loop + its holes = one `SplitFace` fragment.

**Output**: `SplitFace` records with `trimming_loops`, `splitting_edges`, provenance

### 3. Classification (`classify_split_faces_against_operand`)

**Input**: Split faces + original solids

**Process**: For each fragment, evaluate a representative point on the actual surface, then classify against the opposite operand using `classify_point_in_solid` (AABB → ray-casting → nearest-face).

**Output**: Each `SplitFace` gets `classification: Some(Inside|Outside|OnBoundary)`

### 4. Selection (`select_split_faces_for_boolean_op`)

**Input**: Classified split faces + `BooleanOp`

**Rules**:
| Operation | Operand 0 selects       | Operand 1 selects |
|-----------|------------------------|-------------------|
| Common    | Inside ∪ OnBoundary    | Inside            |
| Fuse      | Outside                | Outside           |
| Cut       | Outside ∪ OnBoundary   | OnBoundary        |
| Section   | (not implemented)      | —                 |

### 5. Vertex Merging & Edge Sewing

**Vertex merging**: Clusters fragment vertices within geometric tolerance, producing a canonical ID map.

**Edge sewing**: Identifies shared edges between fragments via merged vertex pairs, builds `SewnEdge` / `SewnPath` records for shell assembly.

### 6. Shell Assembly (`assemble_shells`)

**Input**: Selected split faces + original face geometry + merged vertex map

**Process**:
- Rebuild each fragment into a `Face<C,S>` with proper topology edges
- Group faces into connected components via shared/orientable edge adjacency
- Orient faces consistently within each shell
- Validate `ShellCondition::Closed`

**Output**: `Vec<Shell>` + `ProvenanceMap`

## Key Data Types

### `TrimmingEdgeSource`

```rust
enum TrimmingEdgeSource {
    OriginalBoundaryEdge(EdgeId),  // from input solid boundary
    SectionCurve(SectionCurveId),  // from face-face intersection
    Unattributed,                  // synthesized
}
```

Replaces the previous `section_curve: Option` + `original_edge: Option` dual-field pattern. Eliminates invalid states (both None, or both Some).

### `CanonicalRebuiltEdge`

```rust
enum CanonicalRebuiltEdge {
    OriginalEdge(EdgeId),                           // traceable to input
    SectionSegment(SectionCurveId, VertexId, VertexId), // per-segment identity
    SharedBoundary(VertexId, VertexId),              // shared across faces
    OpenBoundary(FaceId, VertexId, VertexId),        // face-specific
}
```

Used for:
- **Within-face dedup**: Distinguishes segments of the same section curve
- **Cross-face sharing**: `canonical_edges_share_identity()` matches by `SectionCurveId` (vertex IDs differ per UV space)
- **Component grouping**: Determines face adjacency for shell assembly

### `TrimmingLoop`

```rust
struct TrimmingLoop {
    face: FaceId,
    vertex_ids: Vec<VertexId>,    // aligned with uv_points traversal
    edges: Vec<TrimmingEdge>,     // per-segment edges
    uv_points: Vec<Point2>,       // closed polyline (first == last)
    signed_area: f64,
    is_outer: bool,
}
```

**Invariant**: For populated loops, `open_loop_vertex_ids().len() == edges.len()`.

### `SplitFace`

```rust
struct SplitFace {
    original_face: FaceId,
    operand_rank: u8,
    trimming_loops: Vec<TrimmingLoop>,   // outer[0] + holes[1..]
    splitting_edges: Vec<SectionCurveId>,
    representative_point: Option<Point3>,
    classification: Option<PointClassification>,
}
```

## Point Classification

Three-tier classifier in `classify_point_in_solid`:

1. **AABB fast path**: If shell has 6 faces forming an axis-aligned box, use direct containment + axis ray counting
2. **Ray-casting**: Newton's method to solve `surface(u,v) = origin + t * direction` for each face, count forward intersections (odd = inside)
3. **Nearest-face fallback**: Find closest surface point, use surface normal dot product to determine inside/outside

## Provenance

`ProvenanceMap` tracks output → input mapping:
- `faces: Vec<FaceProvenance>` — which input face each output face came from
- `edges: FxHashMap<(VertexId, VertexId), Vec<SourceOrigin>>` — edge source tracking
- `merged_vertices: FxHashMap<VertexId, VertexId>` — vertex merge mapping
