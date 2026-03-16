## truck-bop validation notes

- Synthetic endpoint paves must allocate generated vertex IDs via `BopDs::next_generated_vertex_id()` instead of using fixed sentinel IDs, so endpoint references remain stable and cannot alias unrelated vertices.
- `TrimmingLoop.vertex_ids` stores fragment-local vertex identities aligned with the open polyline returned by dropping the duplicated closing UV sample; topology-rebuild helpers zip those IDs against open polygon vertices, so callers must preserve that invariant when constructing split-face records.
- `build_solids_from_shells` relies on `truck_topology::Solid::new` to enforce shell connectivity, closure, and manifoldness after `truck-bop` has already filtered out non-closed shells.
