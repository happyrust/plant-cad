# Mission: Port OCCT Boolean Operations to truck

## 1. Goal

Port the core architecture of OCCT Boolean Operations (BOP) into `truck-new` as a new
`truck-bop` crate, and progressively replace the current mesh-driven boolean pipeline in
`truck-shapeops` with a more robust B-Rep-oriented implementation.

The mission is not to clone OCCT line by line. The goal is to transplant the successful
architectural model:

- central boolean data structure
- staged intersection pipeline
- edge splitting around parameter-space paves
- face splitting, classification, and topology rebuild
- explicit tolerance management and robust intermediate states

## 2. Why This Mission Exists

The current boolean implementation in `truck-shapeops` is workable but structurally limited.
Its current flow is effectively:

`triangulation -> interference extraction -> polyline construction -> intersection curves -> loop store -> face division -> classification -> integrate`

This design has several constraints:

- it depends on polygonal interference detection as the main source of truth
- it lacks a Pave/PaveBlock-style exact edge splitting model
- it has no OCCT-style staged V/V, V/E, V/F, E/E, E/F, F/F interference data structure
- it exposes only `and` and `or` as stable public boolean operations
- it uses a relatively shallow in/out classification model

In contrast, OCCT organizes boolean operations around `BOPAlgo_PaveFiller` and `BOPDS_DS`,
with robust intermediate data for paves, pave blocks, face states, section edges, and
interferences. That architecture is a better long-term fit for serious B-Rep booleans.

## 3. Scope

### In Scope

- create a new `truck-bop` crate in the workspace
- define a Rust-native boolean data structure inspired by `BOPDS_DS`
- implement a staged intersection pipeline
- implement edge splitting using paves and pave blocks
- implement face splitting and classification
- support `common`, `fuse`, `cut`, and `section`
- rebuild valid `Shell` and `Solid` results for truck topology
- provide tests, regression cases, and a migration path from `truck-shapeops`

### Out of Scope for the First Delivery

- exact parity with all OCCT boolean corner cases
- full analytic NURBS/NURBS face-face intersection from day one
- removal of `truck-shapeops`
- large API-breaking changes across unrelated truck crates

## 4. Source Architecture References

### truck side

- `truck-shapeops/src/transversal/integrate/mod.rs`
- `truck-shapeops/src/transversal/intersection_curve/mod.rs`
- `truck-shapeops/src/transversal/divide_face/mod.rs`
- `truck-shapeops/src/transversal/faces_classification/mod.rs`
- `truck-topology/src/*.rs`

### OCCT side

- `src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller.hxx`
- `src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.hxx`
- `src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PaveBlock.hxx`

## 5. Architectural Decisions

### Decision A: Introduce a new crate

Use a new `truck-bop` crate instead of rewriting `truck-shapeops` in place.

Rationale:

- preserves the existing implementation as a reference baseline
- avoids destabilizing current users during development
- matches truck's modular workspace design
- allows phased migration and A/B validation

### Decision B: Use an index-based DS, not an object graph clone of OCCT

The Rust port should not imitate OCCT's handle-heavy object model. Instead, use typed IDs,
arena-like storage, and explicit indices.

Rationale:

- maps better to Rust ownership and borrowing
- makes intermediate states easier to serialize, debug, and validate
- avoids pervasive interior mutability

### Decision C: Make tolerance explicit and first-class

All major steps must operate through an explicit tolerance policy rather than a loosely passed
single floating-point value.

### Decision D: Use a phased F/F strategy

Face/face intersection is the highest-risk area. The first milestone may reuse the current
truck mesh/polyline-based intersection route as a temporary backend, but the new crate must own
the surrounding DS, split, classification, and rebuild flow.

## 6. Target Public API

```rust
pub fn common<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<Solid<Point3, C, S>, BopError>;

pub fn fuse<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<Solid<Point3, C, S>, BopError>;

pub fn cut<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<Solid<Point3, C, S>, BopError>;

pub fn section<C, S>(
    a: &Solid<Point3, C, S>,
    b: &Solid<Point3, C, S>,
    tol: f64,
) -> Result<Shell<Point3, C, S>, BopError>;
```

## 7. Proposed Crate Layout

```text
truck-bop/
  Cargo.toml
  src/
    lib.rs
    error.rs
    options.rs
    pipeline.rs
    bopds/
      mod.rs
      arena.rs
      ids.rs
      shape_info.rs
      pave.rs
      pave_block.rs
      common_block.rs
      face_info.rs
      interference.rs
    intersect/
      mod.rs
      broad_phase.rs
      vv.rs
      ve.rs
      vf.rs
      ee.rs
      ef.rs
      ff.rs
    split/
      mod.rs
      edge_splitter.rs
      face_splitter.rs
      loops.rs
    classify/
      mod.rs
      point_in_solid.rs
      selector.rs
    build/
      mod.rs
      sewer.rs
      shell_builder.rs
    tests/
      fixtures.rs
      regression.rs
```

## 8. Core Data Model

The Rust data model should preserve OCCT's boolean semantics without preserving its runtime
representation.

### Core identifiers

- `ShapeId`
- `VertexId`
- `EdgeId`
- `FaceId`
- `PaveBlockId`
- `CommonBlockId`
- `SectionCurveId`

### Core entities

```rust
pub struct Pave {
    pub vertex: VertexId,
    pub parameter: f64,
    pub tolerance: f64,
}

pub struct PaveBlock {
    pub edge: EdgeId,
    pub original_edge: EdgeId,
    pub pave0: Pave,
    pub pave1: Pave,
    pub ext_paves: Vec<Pave>,
    pub shrunk_range: Option<(f64, f64)>,
    pub is_splittable: bool,
}

pub struct CommonBlock {
    pub pave_blocks: Vec<PaveBlockId>,
    pub support_faces: Vec<FaceId>,
}
```

### Data structure responsibilities

`BopDs` must own:

- source shapes and generated shapes
- shape metadata and rank/source relationships
- edge-to-pave-block pools
- face state information
- same-domain shape mapping
- all interference tables
- section curves and section edges

## 9. Tolerance Policy

Introduce an explicit options object:

```rust
pub struct BopOptions {
    pub geometric_tol: f64,
    pub parametric_tol: f64,
    pub angular_tol: f64,
    pub approximation_tol: f64,
    pub enable_ff_fallback: bool,
}
```

Rules:

- input topology is not modified in place unless explicitly allowed
- tolerance growth is recorded, not hidden
- vertex, edge, and face local tolerances may exceed the initial global tolerance
- split and classification logic must consume the recorded tolerances consistently

## 10. Pipeline Overview

```text
prepare -> broad phase -> VV/VE/VF/EE/EF/FF -> split edges -> split faces
-> classify fragments -> select by boolean op -> rebuild topology
```

## 11. Mission Breakdown

### Phase 0 - Baseline and Design

#### Objectives

- create the formal architecture baseline
- identify what can be reused from `truck-shapeops`
- confirm the first delivery scope

#### Tasks

- add `truck-bop` to workspace planning
- write an ADR for crate strategy and tolerance model
- build a gap matrix between current truck and target OCCT architecture
- identify reusable geometry and topology helpers from truck crates

#### Deliverables

- this mission file
- a capability matrix
- an ADR for `truck-bop`

### Phase 1 - Crate Bootstrapping and DS

#### Objectives

- create the `truck-bop` crate
- define the Rust boolean data structure
- establish the error and options model

#### Tasks

- create `truck-bop/Cargo.toml`
- update root `Cargo.toml`
- add `lib.rs`, `error.rs`, `options.rs`, and `pipeline.rs`
- implement `BopDs` skeleton and typed IDs
- add first unit tests for DS storage and indexing

#### Exit Criteria

- workspace builds with the new crate
- DS can register source shapes and synthetic test entries

### Phase 2 - Broad Phase and Low-Dimension Intersections

#### Objectives

- produce stable candidate pairs
- generate paves and low-level interference records

#### Tasks

- implement bounding providers and candidate pair generation
- implement `VV`
- implement `VE`
- implement `VF`
- implement `EE`
- implement `EF`
- feed all outputs into `BopDs`

#### Exit Criteria

- low-dimension interferences are queryable from DS
- affected edges accumulate paves

### Phase 3 - PaveBlock Splitting

#### Objectives

- convert edge-local paves into robust split segments

#### Tasks

- sort and deduplicate edge paves
- create `PaveBlock`s
- compute valid and shrunk ranges
- mark unsplittable or micro segments
- support common-block propagation

#### Exit Criteria

- test cases can split edges deterministically
- re-running split is stable and idempotent

### Phase 4 - Face/Face v1 and Section Representation

#### Objectives

- introduce section curves into the DS
- connect F/F output to the new split pipeline

#### Tasks

- implement `intersect/ff.rs`
- initially wrap current truck intersection-curve functionality if needed
- normalize outputs into `SectionCurve` and section-edge records
- attach section endpoints to face and edge structures

#### Exit Criteria

- section curves exist inside `BopDs`
- F/F results can drive face splitting in simple cases

### Phase 5 - Face Splitting

#### Objectives

- split faces into fragments bounded by original edges and section edges

#### Tasks

- project section edges into face parameter space
- build trimming loops
- distinguish outer and inner loops
- create split-face records with provenance

#### Exit Criteria

- test solids produce split faces that can be individually classified

### Phase 6 - Classification and Boolean Selection

#### Objectives

- classify fragments against the opposite operand
- support all target boolean operation modes

#### Tasks

- implement point classification
- define selection logic for `fuse`, `common`, `cut`, and `section`
- add special handling for boundary and same-domain situations

#### Exit Criteria

- boolean selection yields deterministic face fragment sets

### Phase 7 - Topology Rebuild

#### Objectives

- reconstruct valid truck shells and solids from selected fragments

#### Tasks

- merge equivalent vertices
- sew matching edges
- orient faces consistently
- assemble connected components into shells
- build valid solids from closed shells

#### Exit Criteria

- rebuilt results can be consumed as normal truck topology objects

### Phase 8 - Integration and Compatibility Layer

#### Objectives

- expose stable API
- allow incremental adoption from `truck-shapeops`

#### Tasks

- stabilize `common`, `fuse`, `cut`, `section`
- optionally provide bridge functions for `and` and `or`
- add feature-flag or adapter path for legacy comparison

#### Exit Criteria

- consumers can call the new crate directly
- existing workflows can be regression-tested against the old path

### Phase 9 - Validation, Regression, and Performance

#### Objectives

- verify correctness and establish a performance baseline

#### Tasks

- add unit tests for DS, paves, splitters, and selectors
- add end-to-end boolean cases
- compare results against current `truck-shapeops`
- compare representative scenarios against OCCT reference behavior
- profile pair generation, intersection hot paths, and rebuild steps

#### Exit Criteria

- regression suite is stable
- main scenarios complete with acceptable robustness and speed

## 12. Suggested Delivery Order

1. `truck-bop` crate skeleton
2. `BopDs` and typed IDs
3. low-dimension intersections
4. edge splitting with `PaveBlock`
5. temporary F/F integration
6. face splitting
7. classification and selection
8. topology rebuild
9. compatibility layer and regression tests
10. improved analytic or hybrid F/F implementation

## 13. Acceptance Criteria

The mission is successful when the following are true:

- `truck-bop` exists as a standalone workspace crate
- it supports `common`, `fuse`, `cut`, and `section`
- it uses an explicit intermediate boolean DS
- it no longer depends solely on mesh interference as the only boolean state model
- representative boolean cases succeed on closed solids
- results can be triangulated, rendered, and exported without obvious topology breakage
- regression tests cover failure-prone cases such as tangency, near-coincidence, and micro edges

## 14. Test Plan

### Unit tests

- DS indexing and source/new shape bookkeeping
- pave sorting and de-duplication
- pave block splitting
- section curve normalization
- selector truth tables for boolean ops

### Integration tests

- box vs cylinder
- sphere vs sphere
- box cut by cylinder
- coplanar touching boxes
- tangent face contact
- near-coincident edges and micro segments

### Cross-checks

- compare selected scenarios with `truck-shapeops`
- compare reference scenarios with OCCT outputs or expected topology counts

### Manual validation

- render using truck visualization tooling
- export to STEP or another supported format and inspect externally

## 15. Main Risks

### Risk 1: Face/Face intersection dominates schedule

Mitigation:

- allow a temporary fallback implementation
- decouple F/F generation from the rest of the new pipeline

### Risk 2: Tolerance inconsistency causes cascading topology failures

Mitigation:

- define tolerance policy before implementing advanced splitting
- store tolerance changes explicitly in DS

### Risk 3: Truck geometric traits are broad but not uniformly robust

Mitigation:

- define capability assumptions clearly
- surface unsupported cases through `BopError`
- avoid silent `Option`-based failure in the new crate

### Risk 4: Premature compatibility constraints slow architecture cleanup

Mitigation:

- prioritize new internal design first
- add compatibility adapters later

## 16. Milestones

### M0 - Design Ready

- architecture baseline approved
- crate strategy approved
- DS model approved

### M1 - DS and Splitting Ready

- `truck-bop` builds
- low-level interferences and pave blocks work

### M2 - MVP Boolean Pipeline Ready

- temporary F/F in place
- simple `common` and `fuse` cases succeed end-to-end

### M3 - Full Operation Set Ready

- `cut` and `section` supported
- topology rebuild stable on representative solids

### M4 - Validation Ready

- regression suite established
- comparison against old path complete

### M5 - Robustness Upgrade

- improved F/F strategy
- performance and difficult-case stabilization

## 17. Recommended Immediate Next Tasks

1. Add `truck-bop` to the workspace and create the crate skeleton.
2. Write the `BopDs` core types and typed IDs.
3. Add `BopError` and `BopOptions`.
4. Implement the first edge pave accumulation tests.
5. Build a minimal pair-generation and `VV/VE` prototype.

## 18. Phase 1 Execution Checklist

This section turns Phase 1 into an implementation-ready checklist with concrete files,
interfaces, and completion conditions.

### 18.1 Phase 1 Objective

Create a buildable `truck-bop` crate that defines the boolean-operation foundation:

- public crate entry points
- explicit error and options types
- typed IDs for boolean DS entities
- a minimal `BopDs` skeleton with source-shape registration
- unit tests proving indexing and storage behavior

This phase does not implement boolean geometry yet. It creates the internal contract that later
phases will build upon.

### 18.2 Phase 1 Deliverables

- root workspace updated to include `truck-bop`
- `truck-bop` crate builds successfully
- `truck-bop` exposes placeholder public API and internal modules
- `BopDs` can register source vertices, edges, faces, and operands
- initial tests pass for IDs, arenas, and DS bookkeeping

### 18.3 Planned Dependencies

Use `truck-shapeops` and `truck-modeling` as the template for dependency shape.

#### Initial crate dependencies

- `thiserror` for `BopError`
- `derive_more` for light derive support if useful
- `rustc-hash` for fast hash maps if needed
- `truck-base`
- `truck-geotrait`
- `truck-geometry`
- `truck-topology`
- `truck-meshalgo`

#### Dev dependencies

- `truck-modeling` for test fixtures
- `proptest` for DS invariants if needed
- `serde_json` only if snapshot-like test output becomes useful

### 18.4 File-by-File Task Breakdown

#### Task P1-01: Update workspace

File:

- `Cargo.toml`

Changes:

- add `truck-bop` to `[workspace].members`

Done when:

- `cargo metadata` recognizes the new crate

#### Task P1-02: Create crate manifest

File:

- `truck-bop/Cargo.toml`

Changes:

- add package metadata consistent with truck workspace
- define initial dependencies and dev-dependencies
- do not add optional features yet unless they are immediately used

Done when:

- `cargo check -p truck-bop` can parse the manifest

#### Task P1-03: Create crate root

File:

- `truck-bop/src/lib.rs`

Responsibilities:

- crate-level lint settings aligned with neighboring truck crates
- module declarations
- re-exports for the initial public surface
- placeholder public API declarations for future boolean operations

Initial public surface to expose:

- `BopError`
- `BopOptions`
- `BopDs`
- typed ID module exports

Done when:

- the crate builds with empty or stub implementations

#### Task P1-04: Define error model

File:

- `truck-bop/src/error.rs`

Responsibilities:

- define `BopError`
- replace the future need for `Option<T>`-only failure semantics

Initial error variants:

- `InvalidTolerance`
- `UnsupportedGeometry`
- `TopologyInvariantBroken`
- `MissingShape(ShapeId)`
- `InternalInvariant(&'static str)`
- `NotImplemented(&'static str)`

Notes:

- keep variants intentionally small in Phase 1
- prefer semantic variants over transport-style string errors

Done when:

- unit tests or compile-time checks can construct and format the error values

#### Task P1-05: Define options model

File:

- `truck-bop/src/options.rs`

Responsibilities:

- define `BopOptions`
- provide a default constructor or `Default` implementation
- provide validation helpers if needed

Suggested initial shape:

```rust
pub struct BopOptions {
    pub geometric_tol: f64,
    pub parametric_tol: f64,
    pub angular_tol: f64,
    pub approximation_tol: f64,
    pub enable_ff_fallback: bool,
}
```

Required methods:

- `Default`
- `fn validate(&self) -> Result<(), BopError>`

Done when:

- invalid tolerance values are rejected deterministically

#### Task P1-06: Add pipeline placeholder

File:

- `truck-bop/src/pipeline.rs`

Responsibilities:

- define a placeholder orchestration layer for future phases
- describe the pipeline stages as enums or structs rather than implementing geometry now

Suggested contents:

- `enum BooleanOp`
- `struct PipelineInput<'a, C, S>`
- `struct PipelineReport`

Done when:

- the crate has a stable place for future intersection/split/build orchestration

#### Task P1-07: Create `bopds` module root

File:

- `truck-bop/src/bopds/mod.rs`

Responsibilities:

- declare DS-related submodules
- expose `BopDs`
- centralize internal DS imports and exports

Done when:

- `lib.rs` can re-export `BopDs` from this module cleanly

#### Task P1-08: Define typed IDs

File:

- `truck-bop/src/bopds/ids.rs`

Responsibilities:

- define lightweight newtype IDs rather than type aliases
- derive traits needed for hashing, indexing, copy, comparison, and debug output

Initial IDs:

- `ShapeId`
- `VertexId`
- `EdgeId`
- `FaceId`
- `PaveBlockId`
- `CommonBlockId`
- `SectionCurveId`

Recommended pattern:

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub u32);
```

Done when:

- tests verify IDs are cheap, comparable, and hashable

#### Task P1-09: Define generic arena storage

File:

- `truck-bop/src/bopds/arena.rs`

Responsibilities:

- provide a minimal typed arena-like container for DS pools
- support push, get, get_mut, iter, len
- avoid premature complexity such as removal or compaction in Phase 1

Suggested direction:

- `struct Arena<T, Id>` backed by `Vec<T>`
- `push` returns a typed ID

Done when:

- typed pools can be used by `BopDs` without unsafe code

#### Task P1-10: Define shape metadata

File:

- `truck-bop/src/bopds/shape_info.rs`

Responsibilities:

- define source/new shape metadata
- capture enough information for later ranking and provenance

Suggested initial fields:

- `kind: ShapeKind`
- `operand_rank: u8`
- `is_source: bool`
- `parent: Option<ShapeId>`

Supporting enum:

- `ShapeKind::{Vertex, Edge, Face, Shell, Solid, SectionEdge, Unknown}`

Done when:

- DS can record shape provenance without depending on later geometry logic

#### Task P1-11: Define `Pave`

File:

- `truck-bop/src/bopds/pave.rs`

Responsibilities:

- define the immutable logical representation of an edge split point

Fields:

- `vertex: VertexId`
- `parameter: f64`
- `tolerance: f64`

Recommended methods:

- constructor with validation
- comparison helper for sorting by parameter

Done when:

- tests verify ordering and construction behavior

#### Task P1-12: Define `PaveBlock`

File:

- `truck-bop/src/bopds/pave_block.rs`

Responsibilities:

- define the split-edge fragment record
- store endpoints and future split metadata

Initial fields:

- `edge: EdgeId`
- `original_edge: EdgeId`
- `pave0: Pave`
- `pave1: Pave`
- `ext_paves: Vec<Pave>`
- `shrunk_range: Option<(f64, f64)>`
- `is_splittable: bool`

Recommended methods:

- `range_tuple()`
- `append_ext_pave()`
- `sorted_paves()` or equivalent helper

Done when:

- the type can serve as the canonical future input to edge splitting

#### Task P1-13: Define `CommonBlock`

File:

- `truck-bop/src/bopds/common_block.rs`

Responsibilities:

- define a minimal representation of coincident/shared pave blocks

Initial fields:

- `pave_blocks: Vec<PaveBlockId>`
- `support_faces: Vec<FaceId>`

Done when:

- DS has a placeholder structure for later coincidence logic

#### Task P1-14: Define face-state placeholder

File:

- `truck-bop/src/bopds/face_info.rs`

Responsibilities:

- define storage for `On` / `In` information on a face

Initial fields:

- `on_vertices: Vec<VertexId>`
- `in_vertices: Vec<VertexId>`
- `on_pave_blocks: Vec<PaveBlockId>`
- `in_pave_blocks: Vec<PaveBlockId>`

Done when:

- DS has a stable location for face state accumulation

#### Task P1-15: Define interference tables

File:

- `truck-bop/src/bopds/interference.rs`

Responsibilities:

- define typed containers for interference relationships
- do not implement geometric logic yet

Suggested initial contents:

- `InterfVV`, `InterfVE`, `InterfVF`, `InterfEE`, `InterfEF`, `InterfFF`
- `InterferenceTable`

Minimum fields:

- operand IDs involved
- optional produced IDs for created vertices, pave blocks, or section curves

Done when:

- DS can store phase outputs in typed collections

#### Task P1-16: Implement `BopDs`

File:

- `truck-bop/src/bopds/mod.rs`

Responsibilities:

- own arenas for all Phase 1 entities
- provide registration and query methods

Minimum required fields:

- shape metadata pool
- vertex/edge/face metadata pools or index maps
- pave block pool
- common block pool
- face info pool
- interference table

Minimum required methods:

- `new()`
- `with_options(options: BopOptions)` if useful
- `register_vertex_source(...) -> VertexId`
- `register_edge_source(...) -> EdgeId`
- `register_face_source(...) -> FaceId`
- `shape_info(shape_id: ShapeId) -> Option<&ShapeInfo>`
- `face_info(face_id: FaceId) -> Option<&FaceInfo>`

Important constraint:

- for Phase 1, registration methods may store metadata only; they do not need to retain full
  cloned truck topology objects unless that is required by the chosen internal design

Done when:

- unit tests prove DS can register source entities and retrieve metadata reliably

#### Task P1-17: Add public API stubs

File:

- `truck-bop/src/lib.rs`

Responsibilities:

- expose placeholder functions for `common`, `fuse`, `cut`, `section`
- return `Err(BopError::NotImplemented(...))` for now

Rationale:

- establishes the target API now
- lets downstream code compile against a stable surface early

Done when:

- function signatures are visible and compile

#### Task P1-18: Add unit tests

Files:

- `truck-bop/src/bopds/ids.rs`
- `truck-bop/src/bopds/arena.rs`
- `truck-bop/src/bopds/pave.rs`
- `truck-bop/src/bopds/mod.rs`

Test targets:

- ID ordering and hashing
- arena push/get round trip
- pave ordering by parameter
- DS source registration preserves rank and source flags
- invalid options and invalid tolerances are rejected

Done when:

- the crate has a meaningful Phase 1 correctness baseline

### 18.5 Interface Notes for Later Compatibility

To reduce future churn, use these conventions now:

- prefer `Result<T, BopError>` over `Option<T>` for all new public APIs
- keep boolean operation naming in OCCT-style semantics:
  - `common`
  - `fuse`
  - `cut`
  - `section`
- allow an adapter later in `truck-shapeops` mapping:
  - `and -> common`
  - `or -> fuse`

### 18.6 Suggested Phase 1 Work Order

1. `Cargo.toml` workspace update
2. `truck-bop/Cargo.toml`
3. `lib.rs`, `error.rs`, `options.rs`, `pipeline.rs`
4. `bopds/ids.rs` and `bopds/arena.rs`
5. `bopds/shape_info.rs`, `pave.rs`, `pave_block.rs`, `common_block.rs`, `face_info.rs`
6. `bopds/interference.rs`
7. `bopds/mod.rs` with `BopDs`
8. public API stubs
9. unit tests
10. `cargo test -p truck-bop`

### 18.7 Phase 1 Exit Checklist

- [ ] workspace contains `truck-bop`
- [ ] crate builds with strict lints enabled
- [ ] `BopError` exists and is used by public stubs
- [ ] `BopOptions` validates tolerance settings
- [ ] all typed IDs are defined as newtypes
- [ ] arena storage works for typed pools
- [ ] `BopDs` stores source metadata and face-state placeholders
- [ ] unit tests cover DS basics
- [ ] public boolean API surface is visible, even if not implemented
