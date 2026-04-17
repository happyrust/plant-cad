---
name: rust-bop-worker
description: Implements truck-bop sphere-box boolean, seam, trim, and regression features with TDD and Cargo-based verification
---

# Rust BOP Worker

NOTE: Startup and cleanup are handled by `mission-worker-base`. This skill defines the work procedure.

## When to Use This Skill

Use this worker for `truck-bop` features involving:
- periodic seam classification in `fclass2d.rs`
- curved-surface FF / section-root fixes in `intersect/ff.rs`
- trim / dedup / sewing / shell-assembly fixes in `trim.rs`
- real sphere + box example/test fixture work
- `bool_occt_verify` matrix updates and box-box regression preservation
- focused Cargo compile/test repair inside `truck-bop`

## Required Skills

None.

## Work Procedure

1. Read the feature description, mission `AGENTS.md`, and the relevant library docs before changing code.
2. Confirm the exact assertion IDs the feature claims in `features.json`; do not claim more than the feature fulfills.
3. Follow test-first discipline:
   - for behavioral changes or regressions, add or adjust the narrowest failing test first
   - for matrix work, add/update the named `bool_occt_verify` row together with the targeted validator that proves legality / closed-shell / solid-count
   - for section-root work, add/update the targeted section smoke validator before implementation
   - run the targeted failing command before implementation
4. Implement the smallest safe slice that moves the feature forward.
5. Preserve worktree safety:
   - do not revert unrelated edits
   - if unexpected overlapping changes appear outside the approved mission baseline, stop and return to the orchestrator
   - if the mission AGENTS file marks specific dirty `truck-bop` files as approved in-scope baseline, you may continue on top of them and commit the resulting feature work
6. Run cargo verification in layers:
   - always run a targeted command for the feature
   - when using `--exact` on nested Rust unit tests, use the fully qualified path (for example `trim::tests::...`) rather than a bare test name
   - run `cargo test -p truck-bop --no-run` when touching signatures, module wiring, or test layout
   - run `cargo check -p truck-bop` after the implementation slice stabilizes
   - run `cargo run -p truck-bop --example bool_occt_verify` whenever the feature touches sphere-box rows, touching/tangent semantics, or box-box sentinel behavior
   - run `cargo test -p truck-bop` whenever the feature claims crate-level safety or finishes a milestone boundary
7. Keep the mission priorities explicit:
   - first legality / closed-shell / exact solid-count
   - if strict invariant sentinels regress inside the same files you are debugging, restore those gates before continuing with downstream overlap/tangent repair
   - then preserve named box-box sentinels
   - only then tighten toward OCCT topology parity
8. Do not accept fallback as success when the contract requires one closed merged solid:
   - overlap/tangent `fuse` cases that fall back to multi-solid passthrough are failures
   - when debugging an overlap/tangent fuse path, add or preserve a regression that proves passthrough fallback is not masking the real failure
   - do not weaken shell/solid gates to make a row print PASS
9. In the handoff, include exact commands, exit codes, changed files, added tests/example rows, and any blockers or discovered issues.

## Example Handoff

```json
{
  "salientSummary": "Added the real sphere-box overlap/contained matrix rows to `bool_occt_verify`, introduced six exact integration validators for overlap and contained legality, and fixed seam/trim behavior so each targeted case now returns the expected one-solid closed result without fuse fallback.",
  "whatWasImplemented": "Created internal sphere/box fixtures for tests/examples, added named sphere-box overlap and contained rows to `truck-bop/examples/bool_occt_verify.rs`, implemented the corresponding exact Cargo integration tests, and adjusted periodic classification plus trim/rebuild handling so `common`, `fuse`, and `cut` satisfy the legality-first gate for those fixtures while preserving the existing box-box matrix surface.",
  "whatWasLeftUndone": "Tangent/contact semantics and the final box-box regression stabilization feature remain.",
  "verification": {
    "commandsRun": [
      {
        "command": "cargo test -p truck-bop box_sphere_overlap_common_returns_one_closed_solid -- --exact",
        "exitCode": 0,
        "observation": "Overlap common now returns exactly one closed solid for the real sphere-box fixture."
      },
      {
        "command": "cargo test -p truck-bop box_sphere_contained_cut_returns_one_closed_solid -- --exact",
        "exitCode": 0,
        "observation": "Contained cut now returns one closed residual solid."
      },
      {
        "command": "cargo check -p truck-bop",
        "exitCode": 0,
        "observation": "No new compile errors after the sphere-box legality slice stabilized."
      },
      {
        "command": "cargo run -p truck-bop --example bool_occt_verify",
        "exitCode": 0,
        "observation": "Sphere-box overlap/contained rows are present and the required box-box rows still execute in the same matrix run."
      }
    ],
    "interactiveChecks": []
  },
  "tests": {
    "added": [
      {
        "file": "truck-bop/tests/box_sphere.rs",
        "cases": [
          {
            "name": "box_sphere_overlap_common_returns_one_closed_solid",
            "verifies": "overlap common satisfies the legality-first gate on the real sphere-box fixture"
          },
          {
            "name": "box_sphere_contained_cut_returns_one_closed_solid",
            "verifies": "contained cut returns exactly one closed residual solid"
          }
        ]
      }
    ]
  },
  "discoveredIssues": []
}
```

## When to Return to Orchestrator

Return to the orchestrator if:
- the feature requires changing crates outside the approved `truck-bop` mission boundary
- tangent/common/cut semantics are ambiguous for the chosen sphere-box fixture
- the only way to make a required case pass is to weaken shell/solid topology gates
- unexpected overlapping changes appear in mission-touched files
- Cargo validation fails for reasons unrelated to the feature and you need scope guidance
