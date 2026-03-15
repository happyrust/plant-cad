## truck-bop validation notes

- Synthetic endpoint paves must allocate generated vertex IDs via `BopDs::next_generated_vertex_id()` instead of using fixed sentinel IDs, so endpoint references remain stable and cannot alias unrelated vertices.
