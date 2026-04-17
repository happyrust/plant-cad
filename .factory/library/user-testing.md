# User Testing

Validation surface and execution guidance for the `truck-bop` real sphere + box boolean mission.

---

## Validation Surface

**Surface:** Cargo CLI only (`truck-bop` crate)

**Primary commands:**
- `cargo test -p truck-bop --no-run`
- targeted exact tests: `cargo test -p truck-bop <name> -- --exact`
- `cargo check -p truck-bop`
- `cargo test -p truck-bop`
- `cargo run -p truck-bop --example bool_occt_verify`

**Approach:**
- use targeted exact validators as the primary proof for sphere-box legality, closed-shell, and exact solid-count assertions
- use `bool_occt_verify` as the named matrix / regression surface
- use `cargo test -p truck-bop --no-run` before broader runs when signatures or module layout change
- reserve full `cargo test -p truck-bop` for milestone gates or features claiming crate-level safety

## Validation Concurrency

**Max concurrent validators:** 2

**Rationale:**
- this mission has one Cargo-based validation surface, but it contains two useful concurrent lanes:
  - a targeted test lane
  - the example-matrix lane
- Cargo shares the same workspace target directory, so concurrency above 2 is likely to add compile and disk contention rather than signal
- dry-run observations showed the environment is capable of running the surface, but a conservative cap of 2 best balances speed and stability

## Validation Readiness Notes

**Dry-run summary:**
- `cargo test -p truck-bop --example bool_occt_verify --no-run` succeeded
- `cargo run -p truck-bop --example bool_occt_verify` succeeded
- the current matrix already reports box-box PASS/DIFF rows, so the validation path is executable

**Current known mission gap:**
- `bool_occt_verify` does not yet contain the required sphere-box rows; adding them is part of this mission

## Flow Validator Guidance: cli-cargo

- Stay on the Rust/Cargo surface only.
- Do not start any local service or GUI application.
- Evidence must cite exact assertion IDs and the exact command used.
- For sphere-box assertions, prefer named targeted tests such as:
  - `box_sphere_overlap_*`
  - `box_sphere_contained_*`
  - `box_sphere_tangent_*`
  - `box_sphere_section_smoke_runs`
- For regression assertions, use `cargo run -p truck-bop --example bool_occt_verify` and verify the named rows required by the validation contract.
- Treat warnings as non-blocking unless they change exit status or make the row/assertion evidence ambiguous.
