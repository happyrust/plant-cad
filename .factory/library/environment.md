# Environment

Environment variables, external dependencies, and setup notes for the `truck-bop` real sphere + box mission.

**What belongs here:** required env vars, external dependencies, setup quirks.
**What does NOT belong here:** service ports/commands (use `.factory/services.yaml`).

---

## External Dependencies

- None required for this mission.
- No database, cache, HTTP service, or third-party credentials are needed.

## Setup Notes

- Run all commands from the repo root: `D:/work/plant-code/cad/truck-new`
- `truck-bop` is a Cargo workspace member
- validation is Cargo CLI only
