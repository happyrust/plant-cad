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
- Mission-scoped BOPDS migration work that must preserve trim provenance and endpoint identity

## Work Procedure

### 1. Understand the Feature

Read the feature description, preconditions, and expected behavior carefully. Check the mission document and AGENTS.md for context.

### 2. Write Failing Tests First (RED)

Before any implementation:
- Create or update test module with `#[cfg(test)] mod tests`
- Write focused unit tests that verify the expected behavior
- Run tests to confirm they fail: `cargo test -p truck-bop <test_name> -- --exact`
- Document what each test verifies in test names and comments

Exception:
- If the feature is explicitly a migration, regression-alignment, or documentation-hardening task and strong focused coverage already exists, you may use the existing failing test/regression as the RED anchor instead of adding a brand-new test. State that choice explicitly in the handoff.
- If the assigned feature is already red on startup for the exact targeted behavior, treat that regression as the RED anchor first and repair the baseline before layering on broader follow-up changes.
- Revert exploratory edits before handoff when they do not improve an already-red regression anchor, and document the investigated root-cause surfaces so the next worker can restart cleanly.

### 3. Implement Minimally (GREEN)

Write the minimal code to make tests pass:
- Follow truck crate conventions (see existing modules)
- Use appropriate truck-geometry and truck-topology APIs
- Handle errors via `BopError`
- Keep implementations focused and minimal
- Run tests frequently: `cargo test -p truck-bop <test_name>`

Mission-specific expectations for BOPDS migration work:
- Treat `PaveBlock` as the authoritative split unit once the mission introduces block-aware APIs.
- Do not bypass block routing by writing only raw global paves when the feature requires VE/EE/EF to feed `BopDs`.
- Preserve `BopDs::next_generated_vertex_id()` endpoint identity and the existing trim provenance/topology-key invariants.
- Do not claim a feature complete if mission-critical data is still represented by placeholders (for example fake `CommonBlock` membership or sentinel block IDs); return to the orchestrator with the blocker instead.
- After integrating EF or other face-state flows, re-run the pre-existing targeted regressions for endpoint-only or tangent contacts before claiming success.

### 4. Verify with Validators

After tests pass:
- Run `cargo check -p truck-bop` (must pass)
- Run `cargo test -p truck-bop` when the feature or milestone contract requires crate-wide confirmation; otherwise follow the narrower command plan in AGENTS.md and `.factory/services.yaml`
- Run `cargo clippy -p truck-bop` only if the current feature explicitly requires clippy or the repository guidance says linting is in scope. Otherwise follow `.factory/services.yaml`.

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
- The best available implementation still relies on placeholder mission-critical data or leaves the new path effectively dead code
- Cannot achieve expected behavior within reasonable scope

## Environment Notes

- `.factory/init.sh` is a shell script. Run it with `sh` or execute it directly; do not invoke it with `python3`.
- Prefer the repo-local commands from `.factory/services.yaml` when they differ from the generic guidance above.
