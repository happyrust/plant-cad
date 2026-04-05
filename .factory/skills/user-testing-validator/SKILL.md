---
name: user-testing-validator
description: Runs mission command-based validation by dispatching one validation subagent per area plus a cross-area synthesis pass
---

# User-Testing Validator

Use this skill only for mission validation after the implementation features are complete.

## Procedure

1. Read the mission's `validation-contract.md`, `validation-state.json`, `features.json`, `AGENTS.md`, and repo `.factory/services.yaml`.
2. Identify the user-testing areas that apply from `validation-contract.md`.
   - For current truck-bop missions these commonly include targeted command checks for early-stage split logic, targeted command checks for later interference/face-state logic, crate-wide regression commands, and a cross-area consistency pass.
3. Dispatch one validation subagent per area. Each subagent should focus on one area only, run the relevant commands, and record precise evidence.
4. Run or verify the baseline crate commands required by the contract:
   - `cargo test -p truck-bop`
   - `cargo check -p truck-bop`
   - `cargo check --release -p truck-bop`
5. Synthesize the area reports into one user-testing verdict:
   - which assertions pass
   - which assertions fail
   - which assertions remain blocked and why
6. Update `validation-state.json` with evidence-backed statuses before handing control back to the orchestrator.

## Requirements

- Prefer targeted commands during area validation, but include the full crate commands when the contract requires them.
- Record exact commands, exit codes, and brief observations.
- Always include a cross-area synthesis showing whether split facts, identity invariants, and downstream consumers remain consistent across the mission's implementation stages.
- A pre-existing dirty worktree is not, by itself, a blocker for writing mission validation artifacts such as synthesis files or `validation-state.json`; only stop if the validator would need to modify unrelated source files or cannot determine which files are safe evidence outputs.
