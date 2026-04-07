# User Testing

Testing surface and validation approach for truck-bop.

**What belongs here:** Test execution details, validation surface info, and resource constraints for CLI validation.

---

## Validation Surface

**Surface:** Command-line (`cargo`)

**Primary tools:**
- `sh .factory/init.sh`
- `cargo test -p truck-bop <exact-test-name> -- --exact`
- `cargo test -p truck-bop`
- `cargo check -p truck-bop`
- `cargo check --release -p truck-bop`

**Mission-specific approach:**
- Use focused exact tests first for split-edge DS storage, materialization, idempotence, CommonBlock linkage, and report alignment.
- Use pipeline and trim regression anchors as the end-to-end CLI surface for downstream consistency.
- Treat command exit status plus concise test output as the user-visible validation surface for this library mission.
- Avoid modifying source files during validation; validators only record evidence.

## Validation Concurrency

**Max concurrent validators:**
- `3` for check-only validation
- `2` for test-heavy or cold-build validation

**Rationale:**
- This machine has 16 GiB RAM and 10 CPU cores.
- Warm `cargo check` dry runs succeeded quickly, but `cargo` and `rustc` can parallelize internally, so CPU oversubscription is the main risk.
- Using 70 percent of practical headroom favors conservative parallelism for reliability.

**Observed dry-run profile:**
- `sh .factory/init.sh`: pass
- `cargo metadata --no-deps --format-version 1`: pass
- `cargo check -p truck-bop`: pass
- `cargo check --release -p truck-bop`: pass
- Warm-check wrapper footprint during dry run: about 69 MB resident footprint per timed command, but real compile/test memory can be materially higher.

## Flow Validator Guidance: cli

- Stay on the cargo CLI surface; do not open browsers or spawn extra services.
- Prefer `cargo test -p truck-bop -- --list` only when an exact test name is unclear, then rerun with `--exact`.
- Record exact commands, exit codes, and the key observation for each assertion.
- Treat warning noise as non-blocking unless it changes exit status or masks assertion-specific failures.
- Use full `cargo test -p truck-bop` only when the contract or milestone gate requires crate-wide confirmation.
