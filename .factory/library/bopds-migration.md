# BOPDS migration notes

Use this note for the Batch 2+ missions that migrate OCCT-style split facts into `truck-bop`.

## Core idea

- `BopDs` is the single fact source for intersection-driven splitting.
- `PaveBlock` is the lifecycle unit for edge segments.
- VE, EE, and EF should contribute facts through block-aware APIs so ordering, deduplication, and identity remain deterministic.

## Invariants to preserve

- Synthetic endpoint paves must continue to use `BopDs::next_generated_vertex_id()`.
- Existing trim provenance and topology-key behavior must remain stable while BOPDS grows.
- Overlap facts must stay distinguishable from point-contact facts.
- `FaceInfo` pools must preserve `On`, `In`, and `Sc` semantics for future FF work.

## Recommended split pipeline shape

1. Identify the owning edge or edge pair.
2. Append new split facts to the relevant block-aware structure.
3. Merge endpoint paves with extra paves.
4. Sort and deduplicate within tolerance.
5. Create child blocks or paired overlap facts.
6. Expose downstream-ready records for `CommonBlock`, `FaceInfo`, and later trimming work.

## Review hot spots

- any helper that still writes only to a global `paves` collection
- any EE overlap path that degrades into multiple unpaired vertex hits
- any EF path that updates edge facts but leaves `FaceInfo` untouched
- any new split ordering that depends on caller order rather than deterministic sort/dedup logic
