---
name: rust-worker
description: Implements Rust library features with TDD approach
---

# Rust Worker

NOTE: Startup and cleanup are handled by `worker-base`. This skill defines the WORK PROCEDURE.

## When to Use This Skill

Use for implementing Rust library features:
- Intersection algorithms
- Data structure methods
- Geometric computations
- BopDs integration

## Work Procedure

1. **Write Tests First (Red)**
   - Add unit tests to the implementation file
   - Run `cargo test -p truck-bop -- <test_name>` to verify failure
   - Tests must fail before implementation

2. **Implement (Green)**
   - Write minimal code to make tests pass
   - Follow truck conventions (typed IDs, BopError, documentation)
   - Use existing truck-geometry utilities

3. **Verify**
   - Run `cargo test -p truck-bop` (all tests must pass)
   - Run `cargo clippy -p truck-bop` (no warnings)
   - Manually verify BopDs queries if applicable

4. **Commit**
   - Stage changes: `git add <files>`
   - Commit with descriptive message
   - Verify: `git log -1`

## Example Handoff

```json
{
  "salientSummary": "Implemented VV intersection detection with distance check; added 3 unit tests covering coincident, near, and distant vertices; all tests pass, clippy clean.",
  "whatWasImplemented": "intersect_vv() function in truck-bop/src/intersect/vv.rs that computes vertex-vertex distance and creates VVInterference records in BopDs when distance <= geometric_tol. Added VVInterference storage to BopDs.",
  "whatWasLeftUndone": "",
  "verification": {
    "commandsRun": [
      {
        "command": "cargo test -p truck-bop -- vv_intersection",
        "exitCode": 0,
        "observation": "3 tests passed: coincident_vertices, near_vertices, distant_vertices"
      },
      {
        "command": "cargo clippy -p truck-bop",
        "exitCode": 0,
        "observation": "No warnings"
      }
    ],
    "interactiveChecks": []
  },
  "tests": {
    "added": [
      {
        "file": "truck-bop/src/intersect/vv.rs",
        "cases": [
          {"name": "coincident_vertices", "verifies": "Distance 0 creates interference"},
          {"name": "near_vertices", "verifies": "Distance < tol creates interference"},
          {"name": "distant_vertices", "verifies": "Distance > tol no interference"}
        ]
      }
    ]
  },
  "discoveredIssues": []
}
```

## When to Return to Orchestrator

- Missing truck-geometry utilities for intersection computation
- Ambiguous tolerance handling requirements
- BopDs storage structure needs extension beyond current design
