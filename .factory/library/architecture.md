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

## Key Algorithms

- **Geometric Intersections**: Newton-Raphson for projection, subdivision for curve-curve
- **Pave Splitting**: Sort by parameter, deduplicate within tolerance
- **Face Splitting**: UV projection, trimming loops
- **Classification**: Ray casting for point-in-solid
- **Topology Rebuild**: Vertex merging, edge sewing, shell assembly
