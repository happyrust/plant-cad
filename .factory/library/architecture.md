# Architecture

Architectural decisions and patterns for truck-bop.

**What belongs here:** Design patterns, algorithm choices, data structure decisions, OCCT architecture mappings.

---

## OCCT Architecture Mapping

truck-bop follows OCCT's BOPAlgo_PaveFiller and BOPDS_DS architecture:

- **BopDs**: Central data structure (like BOPDS_DS)
- **Paves**: Vertices on edges with parameters
- **PaveBlocks**: Edge segments between paves
- **SplitEdgeRecords**: Lightweight materialized facts derived from final pave blocks
- **Section Curves**: Face-face intersection results
- **Interference Tables**: VV, VE, VF, EE, EF, FF records

## Geometry Notes

- **Trimmed-face containment**: `Face::boundaries()` yields all boundary wires with face orientation applied, including inner loops. Point containment for trimmed faces must therefore use outer-minus-holes logic instead of requiring every loop to contain the point.

## Key Algorithms

- **Geometric Intersections**: Newton-Raphson for projection, subdivision for curve-curve
- **Pave Splitting**: Sort by parameter, deduplicate within tolerance
- **Synthetic pave endpoints**: When an edge is missing endpoint paves, allocate synthetic endpoint vertex IDs through `BopDs::next_generated_vertex_id()` instead of using sentinel constants so endpoint paves remain stable references in downstream topology.
- **Split-edge materialization**: Materialize lightweight split-edge facts from the final `PaveBlock` set only after edge-local splitting has settled. Clear stale materialized facts before rebuilding them.
- **Face Splitting**: UV projection, trimming loops
- **Classification**: Ray casting for point-in-solid
- **Topology Rebuild**: Vertex merging, edge sewing, shell assembly
- **Topology rebuild invariant**: downstream shell reconstruction must operate on fragment-level trimming data, not just `SplitFace.original_face`, because multiple selected fragments can originate from one source face after splitting.
- **Trim provenance refactors**: represent edge provenance and topology identity explicitly in the data model. Prefer enums/key types over scattered `Option` combinations and lossy identity coercion.

## BOPDS Migration Notes

- Treat `PaveBlock` as the lifecycle owner for edge split facts once Batch 2+ migration work starts. `ext_paves` or equivalent mission-specific split facts should stay attached to the block-aware flow rather than leaking into independent side tables.
- VE, EE, and EF migration work should route through `BopDs` APIs that can preserve ordering, deduplication, and identity invariants. Avoid stage-specific shortcuts that write only raw paves and skip the block lifecycle.
- Overlap semantics deserve first-class representation. If a contact is a shared segment, model it as paired split facts or CommonBlock-ready structure rather than degrading it into several unrelated vertex hits.
- W5-2 split-edge materialization should derive overlap context from existing CommonBlock mappings and avoid persisting unstable `PaveBlockId` assumptions in downstream consumers unless the ID contract is made explicit.
- Pipeline/reporting changes should consume the materialized split-edge layer without forcing `trim.rs` to abandon the established provenance or topology-key model.
