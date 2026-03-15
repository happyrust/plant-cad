---
name: rust-bop-worker
description: Implements truck-bop geometric algorithms and data structures with TDD
---

# Rust BOP Worker

NOTE: Startup and cleanup are handled by `mission-worker-base`. This skill defines the WORK PROCEDURE.

## When to Use This Skill

Use this worker for truck-bop features involving:
- Geometric intersection algorithms (VF, EE, EF, FF)
- Pave and PaveBlock splitting logic
- Section curve representation
- Face splitting algorithms
- Classification and selection logic
- Topology rebuild operations

## Work Procedure

### 1. Understand the Feature

Read the feature description, preconditions, and expected behavior carefully. Check the mission document and AGENTS.md for context.

### 2. Write Failing Tests First (RED)

Before any implementation:
- Create or update test module with `#[cfg(test)] mod tests`
- Write focused unit tests that verify the expected behavior
- Run tests to confirm they fail: `cargo test -p truck-bop <test_name> -- --exact`
- Document what each test verifies in test names and comments

### 3. Implement Minimally (GREEN)

Write the minimal code to make tests pass:
- Follow truck crate conventions (see existing modules)
- Use appropriate truck-geometry and truck-topology APIs
- Handle errors via `BopError`
- Keep implementations focused and minimal
- Run tests frequently: `cargo test -p truck-bop <test_name>`

### 4. Verify with Validators

After tests pass:
- Run `cargo check -p truck-bop` (must pass)
- Run `cargo test -p truck-bop` (all tests must pass)
- Run `cargo clippy -p truck-bop` (address warnings)

### 5. Manual Verification

For geometric algorithms:
- Create a simple test case in `truck-bop/examples/` if helpful
- Verify the algorithm produces sensible results
- Check edge cases (degenerate geometry, tolerance boundaries)

### 6. Document in Handoff

Capture all verification steps and results in the handoff.

## Example Handoff

```json
{
  "salientSummary": "Implemented VF intersection using surface projection and boundary checking; added 5 unit tests covering on-surface, off-surface, and boundary cases; all tests pass, cargo check clean.",
  "whatWasImplemented": "Added intersect_vf function in truck-bop/src/intersect/vf.rs with Newton-Raphson surface projection, parameter bounds checking, and tolerance-based distance verification. Updated BopDs to store VF interference records.",
  "whatWasLeftUndone": "",
  "verification": {
    "commandsRun": [
      {
        "command": "cargo test -p truck-bop vf_intersection",
        "exitCode": 0,
        "observation": "5 tests passed: on_surface, off_surface, at_boundary, outside_bounds, tolerance_edge_case"
      },
      {
        "command": "cargo check -p truck-bop",
        "exitCode": 0,
        "observation": "No errors, clean build"
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
        "file": "truck-bop/src/intersect/vf.rs",
        "cases": [
          {"name": "vf_intersection_detects_vertex_on_surface", "verifies": "Vertex within tolerance of surface is detected"},
          {"name": "vf_intersection_ignores_far_vertex", "verifies": "Vertex beyond tolerance is ignored"},
          {"name": "vf_intersection_checks_parameter_bounds", "verifies": "Projection outside face bounds is rejected"},
          {"name": "vf_intersection_handles_boundary", "verifies": "Vertex on face boundary is detected"},
          {"name": "vf_intersection_tolerance_edge_case", "verifies": "Vertex exactly at tolerance distance"}
        ]
      }
    ]
  },
  "discoveredIssues": []
}
```

## When to Return to Orchestrator

Return to orchestrator if:
- Feature depends on unimplemented functionality in other modules
- Requirements are ambiguous or contradictory
- Geometric algorithm needs research or external reference
- Tests reveal fundamental design issues
- Cannot achieve expected behavior within reasonable scope
