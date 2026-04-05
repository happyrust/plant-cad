## truck-bop validation notes

- Synthetic endpoint paves must allocate generated vertex IDs via `BopDs::next_generated_vertex_id()` instead of using fixed sentinel IDs, so endpoint references remain stable and cannot alias unrelated vertices.
- `TrimmingLoop.vertex_ids` stores fragment-local vertex identities aligned with the open polyline returned by dropping the duplicated closing UV sample; topology-rebuild helpers zip those IDs against open polygon vertices, so callers must preserve that invariant when constructing split-face records.
- `build_solids_from_shells` relies on `truck_topology::Solid::new` to enforce shell connectivity, closure, and manifoldness after `truck-bop` has already filtered out non-closed shells.
- Trim provenance should be modeled explicitly. Avoid reintroducing the legacy `section_curve: Option<_>` plus `original_edge: Option<_>` encoding when a typed provenance enum can make source-boundary, section-curve, and generated-edge cases unambiguous.
- Shared trim/topology identity should use a unified key type across trimming, sewing, and rebuild. Do not collapse `SectionCurveId` into lossy `EdgeId` fallbacks in new code.
- The trim provenance migration is complete: preserve the explicit `TrimmingEdgeProvenance` categories (`SourceBoundary`, `SectionCurve`, `Generated`) and the shared `TrimmingTopologyKey` model across trim construction, sewing, and rebuild.
- Generated topology keys are intentionally fragment-scoped (`face`, `loop_index`, `edge_index`); do not treat them as reusable shared topology across faces.
- `cyclic_edge_sequence_matches()` should compare reversed cyclic orders by index arithmetic rather than allocating a reversed temporary `Vec`.
- Regression anchors for this model include source-boundary identity reuse, section-curve identity preservation on sewn open paths, generated-edge locality, shell assembly grouping, shell orientation repair, and disconnected-source-face reuse checks.
- Current BOPDS migration work should keep split facts flowing through `PaveBlock`-aware `BopDs` APIs. If VE, EE, or EF learns new facts, those facts must not bypass the edge-block lifecycle and create a second ordering or identity source of truth.
- For overlap-heavy work, prefer explicit paired-block or CommonBlock-ready facts over collapsing everything into point hits; later trimming and FF work depends on that distinction surviving the migration.
