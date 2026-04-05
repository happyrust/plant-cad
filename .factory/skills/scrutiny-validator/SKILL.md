---
name: scrutiny-validator
description: Runs mission scrutiny validation by dispatching one review subagent per area plus a cross-area synthesis pass
---

# Scrutiny Validator

Use this skill only for mission validation after all implementation features in the milestone or mission are complete.

## Procedure

1. Read the mission's `validation-contract.md`, `validation-state.json`, `features.json`, and `AGENTS.md`.
2. Identify the scrutiny areas that apply from `validation-contract.md`.
   - For current truck-bop missions these commonly include data-model or PaveBlock lifecycle checks, intersection-fact routing checks, face-state or rebuild checks, regression/docs checks, and a cross-area consistency pass.
3. Dispatch one review subagent per area. Prefer specialist review subagents when available; otherwise use a general review-capable worker. Each subagent must focus on one area only.
4. Collect the subagent findings and map them back to the relevant validation assertion IDs.
5. Synthesize the area results into one scrutiny verdict:
   - which assertions pass
   - which assertions fail
   - which assertions remain blocked and why
6. Update `validation-state.json` with evidence-backed statuses before handing control back to the orchestrator.

## Requirements

- Do not re-implement features during scrutiny unless the orchestrator explicitly turns the findings into a new pending feature.
- Keep findings actionable and tied to files/tests/behaviors.
- Always include a cross-area synthesis that checks whether the fact model remains coherent from the earliest split-producing stage through the latest downstream consumer covered by the mission.
