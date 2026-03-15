# Architecture

Architectural decisions and patterns for truck-bop Phase 2.

**What belongs here:** Design patterns, intersection algorithm approaches, BopDs integration patterns.

---

## Intersection Pipeline Architecture

Phase 2 uses a staged pipeline:
1. Broad phase: Generate candidate pairs by bbox overlap
2. Narrow phase: Run type-specific intersection algorithms
3. Storage: Write results to BopDs

## Tolerance Strategy

All intersection algorithms use `BopOptions.geometric_tol` consistently.
Never hardcode tolerance values.
