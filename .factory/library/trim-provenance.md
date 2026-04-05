# Trim provenance refactor notes

Use this note when working on the mission that formalizes trim provenance in `truck-bop`.

## Desired end state

- `TrimmingEdge` expresses provenance explicitly instead of encoding meaning through paired `Option` fields.
- A unified topology-key model is shared across trim construction, sewing, and rebuild.
- `SewnEdgeSource` carries topology identity without coercing section-curve identity into `EdgeId`.
- `TrimmingLoop.vertex_ids` keeps its existing alignment contract with the open traversal consumed by rebuild helpers.

## Migration boundaries

- Safe scope:
  - `truck-bop/src/bopds/interference.rs`
  - `truck-bop/src/trim.rs`
  - direct tests and fixtures that construct `TrimmingEdge` or `SewnEdgeSource`
  - plan/library docs that explain the new model
- Avoid broad workspace cleanup or unrelated topology rewrites.

## Regression anchors

Keep the repaired trim/shell pipeline behavior from `docs/plans/2026-03-28-truck-bop-trim-pipeline-repair-v2.md` as the baseline. The provenance refactor is allowed to clarify and strengthen identity handling, but it must not weaken:

- shell assembly grouping
- shell closure checks
- shell orientation validation
- split-face reuse rules
- trimming-loop reversal/alignment behavior

## Review hot spots

- any code that previously matched on `section_curve.is_some()` or `original_edge.is_none()`
- any code that used `original_edge.or(section_curve.map(...))`
- any constructor or fixture that can accidentally reintroduce impossible provenance states

## Cross-mission note

Later BOPDS migration work must preserve this provenance model while expanding split facts. New VE/EE/EF or PaveBlock lifecycle code should feed downstream trimming data without reintroducing lossy identity fallbacks or duplicate vertex-id sources.
