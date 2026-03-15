# Truck BOP Phase 1 Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create the initial `truck-bop` crate, wire it into the workspace, and land the minimal data-structure foundation for future boolean-operation development.

**Architecture:** Build `truck-bop` as a new workspace crate instead of modifying `truck-shapeops` in place. Phase 1 only establishes the crate boundary, public API stubs, explicit error/options types, typed IDs, and a minimal `BopDs` storage model so later phases can add intersections and splitting without reworking the foundation.

**Tech Stack:** Rust 2021, Cargo workspace, `thiserror`, `derive_more`, `truck-base`, `truck-geotrait`, `truck-geometry`, `truck-topology`, `truck-meshalgo`, `truck-modeling`, `proptest`

---

### Task 1: Add `truck-bop` to the workspace

**Files:**
- Modify: `Cargo.toml`

**Step 1: Write the failing test**

There is no Rust test yet. Use Cargo metadata as the first failing check.

**Step 2: Run test to verify it fails**

Run: `cargo metadata --no-deps`
Expected: The workspace does not list `truck-bop`.

**Step 3: Write minimal implementation**

Add `"truck-bop"` to `[workspace].members` in `Cargo.toml`.

**Step 4: Run test to verify it passes**

Run: `cargo metadata --no-deps`
Expected: The workspace package list includes `truck-bop` once the crate exists.

**Step 5: Commit**

```bash
git add Cargo.toml
git commit -m "build: add truck-bop workspace member"
```

### Task 2: Create the crate manifest

**Files:**
- Create: `truck-bop/Cargo.toml`

**Step 1: Write the failing test**

Use Cargo parse/build failure as the test.

**Step 2: Run test to verify it fails**

Run: `cargo check -p truck-bop`
Expected: FAIL because `truck-bop/Cargo.toml` does not exist.

**Step 3: Write minimal implementation**

Create `truck-bop/Cargo.toml` with package metadata and these sections:

```toml
[package]
name = "truck-bop"
version = "0.1.0"
edition = "2021"
description = "Boolean operation infrastructure for truck"
homepage = "https://github.com/ricosjp/truck"
repository = "https://github.com/ricosjp/truck"
license = "Apache-2.0"

[dependencies]
derive_more = { version = "2.1.1", features = ["full"] }
thiserror = "2.0.18"
rustc-hash = "2.1.1"
truck-base = { version = "0.5.0", path = "../truck-base" }
truck-geotrait = { version = "0.4.0", path = "../truck-geotrait" }
truck-geometry = { version = "0.5.0", path = "../truck-geometry" }
truck-topology = { version = "0.6.0", path = "../truck-topology" }
truck-meshalgo = { version = "0.4.0", path = "../truck-meshalgo" }

[dev-dependencies]
proptest = "1.10.0"
truck-modeling = { version = "0.6.0", path = "../truck-modeling" }
```

**Step 4: Run test to verify it passes**

Run: `cargo check -p truck-bop`
Expected: Cargo reads the manifest, then fails later because source files do not exist yet.

**Step 5: Commit**

```bash
git add truck-bop/Cargo.toml
git commit -m "build: add truck-bop crate manifest"
```

### Task 3: Create the crate root and public module skeleton

**Files:**
- Create: `truck-bop/src/lib.rs`
- Create: `truck-bop/src/error.rs`
- Create: `truck-bop/src/options.rs`
- Create: `truck-bop/src/pipeline.rs`
- Create: `truck-bop/src/bopds/mod.rs`

**Step 1: Write the failing test**

Use crate compilation as the test.

**Step 2: Run test to verify it fails**

Run: `cargo check -p truck-bop`
Expected: FAIL because `src/lib.rs` is missing.

**Step 3: Write minimal implementation**

Create `truck-bop/src/lib.rs` with truck-style lints, module declarations, and re-exports:

```rust
#![cfg_attr(not(debug_assertions), deny(warnings))]
#![deny(clippy::all, rust_2018_idioms)]
#![warn(
    missing_docs,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

mod bopds;
mod error;
mod options;
mod pipeline;

pub use bopds::{BopDs, CommonBlockId, EdgeId, FaceId, PaveBlockId, SectionCurveId, ShapeId, VertexId};
pub use error::BopError;
pub use options::BopOptions;
pub use pipeline::BooleanOp;
```

Create placeholder `error.rs`, `options.rs`, `pipeline.rs`, and `bopds/mod.rs` with enough stubs to compile.

**Step 4: Run test to verify it passes**

Run: `cargo check -p truck-bop`
Expected: The crate compiles or fails only on missing definitions inside the newly created files.

**Step 5: Commit**

```bash
git add truck-bop/src/lib.rs truck-bop/src/error.rs truck-bop/src/options.rs truck-bop/src/pipeline.rs truck-bop/src/bopds/mod.rs
git commit -m "feat: add truck-bop module skeleton"
```

### Task 4: Define `BopError`

**Files:**
- Modify: `truck-bop/src/error.rs`
- Test: `truck-bop/src/error.rs`

**Step 1: Write the failing test**

Add a unit test in `error.rs`:

```rust
#[test]
fn displays_not_implemented_error() {
    let err = BopError::NotImplemented("phase-1 stub");
    assert!(err.to_string().contains("phase-1 stub"));
}
```

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop displays_not_implemented_error -- --exact`
Expected: FAIL because `BopError` is not fully defined.

**Step 3: Write minimal implementation**

Define:

```rust
#[derive(Debug, thiserror::Error)]
pub enum BopError {
    #[error("invalid tolerance configuration")]
    InvalidTolerance,
    #[error("unsupported geometry")]
    UnsupportedGeometry,
    #[error("topology invariant broken")]
    TopologyInvariantBroken,
    #[error("missing shape: {0:?}")]
    MissingShape(crate::ShapeId),
    #[error("internal invariant violated: {0}")]
    InternalInvariant(&'static str),
    #[error("not implemented: {0}")]
    NotImplemented(&'static str),
}
```

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop displays_not_implemented_error -- --exact`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/error.rs
git commit -m "feat: add truck-bop error model"
```

### Task 5: Define `BopOptions`

**Files:**
- Modify: `truck-bop/src/options.rs`
- Test: `truck-bop/src/options.rs`

**Step 1: Write the failing test**

Add tests:

```rust
#[test]
fn default_options_validate() {
    BopOptions::default().validate().unwrap();
}

#[test]
fn rejects_non_positive_tolerance() {
    let opts = BopOptions { geometric_tol: 0.0, ..BopOptions::default() };
    assert!(opts.validate().is_err());
}
```

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop default_options_validate rejects_non_positive_tolerance`
Expected: FAIL because `BopOptions` and `validate()` are incomplete.

**Step 3: Write minimal implementation**

Implement `BopOptions` with `Default` and `validate()` returning `Result<(), BopError>`.

Use sensible defaults such as:

```rust
impl Default for BopOptions {
    fn default() -> Self {
        Self {
            geometric_tol: 1.0e-6,
            parametric_tol: 1.0e-8,
            angular_tol: 1.0e-10,
            approximation_tol: 1.0e-4,
            enable_ff_fallback: true,
        }
    }
}
```

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop default_options_validate rejects_non_positive_tolerance`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/options.rs
git commit -m "feat: add truck-bop options model"
```

### Task 6: Add pipeline placeholders and operation enum

**Files:**
- Modify: `truck-bop/src/pipeline.rs`
- Test: `truck-bop/src/pipeline.rs`

**Step 1: Write the failing test**

Add a unit test:

```rust
#[test]
fn boolean_op_debug_name_is_stable() {
    assert_eq!(format!("{:?}", BooleanOp::Common), "Common");
}
```

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop boolean_op_debug_name_is_stable -- --exact`
Expected: FAIL because `BooleanOp` is missing or incomplete.

**Step 3: Write minimal implementation**

Define:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanOp {
    Fuse,
    Common,
    Cut,
    Section,
}

#[derive(Debug)]
pub struct PipelineReport;
```

Add a placeholder `PipelineInput<'a, C, S>` if needed for future coordination.

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop boolean_op_debug_name_is_stable -- --exact`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/pipeline.rs
git commit -m "feat: add boolean pipeline placeholders"
```

### Task 7: Define typed IDs

**Files:**
- Create: `truck-bop/src/bopds/ids.rs`
- Modify: `truck-bop/src/bopds/mod.rs`
- Modify: `truck-bop/src/lib.rs`
- Test: `truck-bop/src/bopds/ids.rs`

**Step 1: Write the failing test**

Add tests:

```rust
#[test]
fn typed_ids_are_orderable() {
    assert!(VertexId(1) < VertexId(2));
}

#[test]
fn typed_ids_are_hashable() {
    let mut set = std::collections::HashSet::new();
    set.insert(EdgeId(7));
    assert!(set.contains(&EdgeId(7)));
}
```

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop typed_ids_are_orderable typed_ids_are_hashable`
Expected: FAIL because the ID types do not exist.

**Step 3: Write minimal implementation**

Create `ids.rs` with newtypes:

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ShapeId(pub u32);
```

Repeat for:

- `VertexId`
- `EdgeId`
- `FaceId`
- `PaveBlockId`
- `CommonBlockId`
- `SectionCurveId`

Re-export them through `bopds/mod.rs` and `lib.rs`.

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop typed_ids_are_orderable typed_ids_are_hashable`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/bopds/ids.rs truck-bop/src/bopds/mod.rs truck-bop/src/lib.rs
git commit -m "feat: add typed ids for truck bop data structures"
```

### Task 8: Implement a typed arena container

**Files:**
- Create: `truck-bop/src/bopds/arena.rs`
- Modify: `truck-bop/src/bopds/mod.rs`
- Test: `truck-bop/src/bopds/arena.rs`

**Step 1: Write the failing test**

Add a unit test using a local test ID type:

```rust
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct TestId(u32);

#[test]
fn arena_push_and_get_round_trip() {
    let mut arena = Arena::<&'static str, TestId>::new(TestId);
    let id = arena.push("alpha");
    assert_eq!(arena.get(id), Some(&"alpha"));
}
```

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop arena_push_and_get_round_trip -- --exact`
Expected: FAIL because `Arena` does not exist.

**Step 3: Write minimal implementation**

Implement a simple typed arena backed by `Vec<T>` with:

- `new(id_from_index: fn(u32) -> Id)`
- `push(value: T) -> Id`
- `get(id: Id) -> Option<&T>`
- `get_mut(id: Id) -> Option<&mut T>`
- `iter()`
- `len()`

Avoid removal or generational indexing in this phase.

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop arena_push_and_get_round_trip -- --exact`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/bopds/arena.rs truck-bop/src/bopds/mod.rs
git commit -m "feat: add typed arena for bop data storage"
```

### Task 9: Add DS domain types

**Files:**
- Create: `truck-bop/src/bopds/shape_info.rs`
- Create: `truck-bop/src/bopds/pave.rs`
- Create: `truck-bop/src/bopds/pave_block.rs`
- Create: `truck-bop/src/bopds/common_block.rs`
- Create: `truck-bop/src/bopds/face_info.rs`
- Create: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/bopds/mod.rs`

**Step 1: Write the failing test**

Add focused tests:

```rust
#[test]
fn pave_orders_by_parameter() {
    let a = Pave::new(VertexId(0), 0.25, 1.0e-6).unwrap();
    let b = Pave::new(VertexId(1), 0.75, 1.0e-6).unwrap();
    assert!(a.parameter < b.parameter);
}
```

and

```rust
#[test]
fn face_info_starts_empty() {
    let info = FaceInfo::default();
    assert!(info.on_vertices.is_empty());
    assert!(info.in_pave_blocks.is_empty());
}
```

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop pave_orders_by_parameter face_info_starts_empty`
Expected: FAIL because the DS domain types do not exist.

**Step 3: Write minimal implementation**

Implement:

- `ShapeKind` and `ShapeInfo`
- `Pave` with `new(...) -> Result<Self, BopError>`
- `PaveBlock`
- `CommonBlock`
- `FaceInfo` with `Default`
- typed interference structs and `InterferenceTable`

Keep these types intentionally small and data-only.

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop pave_orders_by_parameter face_info_starts_empty`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/bopds/shape_info.rs truck-bop/src/bopds/pave.rs truck-bop/src/bopds/pave_block.rs truck-bop/src/bopds/common_block.rs truck-bop/src/bopds/face_info.rs truck-bop/src/bopds/interference.rs truck-bop/src/bopds/mod.rs
git commit -m "feat: add foundational bop data model types"
```

### Task 10: Implement `BopDs`

**Files:**
- Modify: `truck-bop/src/bopds/mod.rs`
- Test: `truck-bop/src/bopds/mod.rs`

**Step 1: Write the failing test**

Add tests:

```rust
#[test]
fn registers_source_entities_with_rank() {
    let mut ds = BopDs::new();
    let vertex = ds.register_vertex_source(0);
    let edge = ds.register_edge_source(1);
    let face = ds.register_face_source(1);

    assert!(ds.vertex_shape_id(vertex).is_some());
    assert!(ds.edge_shape_id(edge).is_some());
    assert!(ds.face_shape_id(face).is_some());
}
```

and

```rust
#[test]
fn returns_shape_info_for_registered_face() {
    let mut ds = BopDs::new();
    let face = ds.register_face_source(1);
    let shape_id = ds.face_shape_id(face).unwrap();
    let info = ds.shape_info(shape_id).unwrap();
    assert!(info.is_source);
}
```

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop registers_source_entities_with_rank returns_shape_info_for_registered_face`
Expected: FAIL because `BopDs` behavior is incomplete.

**Step 3: Write minimal implementation**

Implement `BopDs` with:

- an owned `BopOptions`
- typed arenas or vectors for `ShapeInfo`, `FaceInfo`, `PaveBlock`, `CommonBlock`
- mapping from `VertexId`/`EdgeId`/`FaceId` to `ShapeId`
- registration methods:
  - `register_vertex_source(operand_rank: u8) -> VertexId`
  - `register_edge_source(operand_rank: u8) -> EdgeId`
  - `register_face_source(operand_rank: u8) -> FaceId`
- query helpers:
  - `shape_info(shape_id: ShapeId) -> Option<&ShapeInfo>`
  - `face_info(face_id: FaceId) -> Option<&FaceInfo>`
  - `vertex_shape_id(vertex_id: VertexId) -> Option<ShapeId>`
  - `edge_shape_id(edge_id: EdgeId) -> Option<ShapeId>`
  - `face_shape_id(face_id: FaceId) -> Option<ShapeId>`

Do not store full truck topology objects yet.

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop registers_source_entities_with_rank returns_shape_info_for_registered_face`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/bopds/mod.rs
git commit -m "feat: add initial bop data structure registry"
```

### Task 11: Add public boolean API stubs

**Files:**
- Modify: `truck-bop/src/lib.rs`
- Test: `truck-bop/src/lib.rs`

**Step 1: Write the failing test**

Add a compilation-level unit test that calls a helper returning the stub error:

```rust
#[test]
fn common_stub_reports_not_implemented() {
    let err = not_implemented_common_error();
    assert!(matches!(err, BopError::NotImplemented(_)));
}
```

If direct generic boolean functions are awkward to unit-test before topology wiring, add a small internal helper.

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop common_stub_reports_not_implemented -- --exact`
Expected: FAIL because the helper or stubs do not exist.

**Step 3: Write minimal implementation**

Add public stubs in `lib.rs`:

```rust
pub fn common<C, S>(
    _a: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _b: &truck_topology::Solid<truck_base::cgmath64::Point3, C, S>,
    _tol: f64,
) -> Result<truck_topology::Solid<truck_base::cgmath64::Point3, C, S>, BopError> {
    Err(BopError::NotImplemented("common"))
}
```

Repeat for `fuse`, `cut`, and `section`.

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop common_stub_reports_not_implemented -- --exact`
Expected: PASS

**Step 5: Commit**

```bash
git add truck-bop/src/lib.rs
git commit -m "feat: add public truck-bop boolean api stubs"
```

### Task 12: Run the full Phase 1 validation

**Files:**
- Verify: `Cargo.toml`
- Verify: `truck-bop/Cargo.toml`
- Verify: `truck-bop/src/*.rs`
- Verify: `truck-bop/src/bopds/*.rs`

**Step 1: Write the failing test**

The failing check is the full package test run before all code exists.

**Step 2: Run test to verify it fails**

Run: `cargo test -p truck-bop`
Expected: FAIL at the current unfinished state.

**Step 3: Write minimal implementation**

Finish any remaining compile errors, lint issues, missing docs, or broken test imports.

**Step 4: Run test to verify it passes**

Run: `cargo test -p truck-bop`
Expected: PASS

Also run:

Run: `cargo check -p truck-bop`
Expected: PASS

**Step 5: Commit**

```bash
git add Cargo.toml truck-bop
git commit -m "feat: bootstrap truck-bop phase 1 foundation"
```
