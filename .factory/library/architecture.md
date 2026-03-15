# Architecture

Architectural decisions and patterns for truck-bop.

**What belongs here:** Design patterns, algorithm choices, data structure decisions, OCCT architecture mappings.

---

## OCCT Architecture Mapping

truck-bop follows OCCT's BOPAlgo_PaveFiller and BOPDS_DS architecture:

- **BopDs**: Central data structure (like BOPDS_DS)
- **Paves**: Vertices on edges with parameters
- **PaveBlocks**: Edge segments between paves
- **Section Curves**: Face-face intersection results
- **Interference Tables**: VV, VE, VF, EE, EF, FF records

## Geometry Notes

- **Trimmed-face containment**: `Face::boundaries()` yields all boundary wires with face orientation applied, including inner loops. Point containment for trimmed faces must therefore use outer-minus-holes logic instead of requiring every loop to contain the point.

## Key Algorithms

- **Geometric Intersections**: Newton-Raphson for projection, subdivision for curve-curve
- **Pave Splitting**: Sort by parameter, deduplicate within tolerance
- **Synthetic pave endpoints**: When an edge is missing endpoint paves, allocate synthetic endpoint vertex IDs through `BopDs::next_generated_vertex_id()` instead of using sentinel constants so endpoint paves remain stable references in downstream topology.
- **Face Splitting**: UV projection, trimming loops
- **Classification**: Ray casting for point-in-solid
- **Topology Rebuild**: Vertex merging, edge sewing, shell assembly
