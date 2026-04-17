# Change Log

The version is of the bottom crate `truck-rendimpl`.

## Unreleased

### truck-bop

- **feat(pipeline):** General ray-casting point classifier (`classify_point_by_ray_casting`) — three-tier strategy: AABB → ray casting → nearest-face fallback. Enables boolean operations on non-axis-aligned and curved solids.
- **feat(bopds):** Complete `CommonBlock` implementation with multi-PaveBlock tracking, face association, and PB↔CB mapping in `BopDs`.
- **feat(bopds):** `FaceInfo` expanded to In/On/Section three-way classification, matching OCCT `BOPDS_FaceInfo` architecture.
- **feat(bopds):** `PaveBlock` extended with ExtPave intermediate split points, `update()` sub-block generation, and `split_edge` tracking.
- **fix(intersect):** `ef.rs` `point_projects_inside_face` now properly distinguishes outer wire from holes, aligned with `vf.rs`.
- **fix(pipeline):** `classify_point_by_nearest_face` merged redundant second shell traversal into single pass.
- **fix(trim):** `classify_loops` rewritten with containment-tree algorithm supporting multi-level nesting (island-in-hole-in-face).
- **feat(api):** Top-level `common`/`fuse`/`cut` functions wired to full boolean pipeline: shape registration → broad phase → 6-level intersection → trim → classify → select → merge → sew → assemble → build solids.
- **feat(intersect):** Analytical plane-plane intersection fast path with 3D line-segment clipping and local tangent-plane projection for face containment.
- **feat(trim):** Edge-graph loop construction with left-turn algorithm for face splitting by open section curves.
- **feat(trim):** `TopologyCache` for shared vertex/edge objects across rebuilt faces, with coordinate-based dedup and canonical ID edge lookup.
- **fix(trim):** Filter boundary-coincident section curves that lie on a face's existing boundary edges, preventing degenerate face splits in adjacent/touching solid configurations.
- **fix(trim):** Fuse selection now keeps only Outside-classified faces (excludes OnBoundary), eliminating duplicate internal surfaces at shared boundaries.
- **fix(trim):** Common selection keeps OnBoundary faces from operand 0 only, avoiding doubled geometry for identical/coincident inputs.
- **fix(trim):** Multi-operand assembly forces face rebuild through `TopologyCache` to merge cross-operand vertices by coordinate proximity.
- **fix(pipeline):** Identical-shape fallback: when Fuse/Common selection yields empty but all faces are OnBoundary, return first operand directly.
- **feat(provenance):** New `provenance` module with `SourceOrigin`, `ProvenanceMap`, `EdgeProvenance`, `FaceProvenance` types — full edge/face origin tracking through boolean operations without modifying `truck-topology` core types.
- **feat(api):** `common_with_provenance`/`fuse_with_provenance`/`cut_with_provenance` API variants returning `BooleanResult { solids, provenance }`.
- **feat(trim):** `assemble_shells` now returns `(shells, ProvenanceMap)`, threading edge/face provenance through `TopologyCache` rebuild and original-face reuse paths.
- **feat(fclass2d):** New `FClass2d` UV-space point-face classifier with multi-wire support, periodic parameter domain handling, and caching; replaces simplified `uv_inside_face` in ray-casting pipeline.
- **fix(trim):** Fuse selection now correctly keeps OnBoundary faces from operand 0 only, aligned with OCCT semantics (previously dropped all OnBoundary).
- **refactor(bopds):** `VertexIdAllocator` type-safe wrapper replacing raw `&mut u32` through vertex ID allocation chain.
- **feat(trim):** Boundary edges in UV edge-graph now sample along curve with 4 intermediate points, improving curved surface accuracy.
- **feat(api):** `section()` operation implemented — extracts FF intersection curves as open shell with BSpline edges.
- **fix(trim):** Accept `Oriented` and `Regular` shells in `build_solids_from_shells`, fixing `overlapping_fuse` and `overlapping_cut` that previously returned `TopologyInvariantBroken`.
- **fix(trim):** `TopologyCache` vertex snap tolerance relaxed to 10× geometric tolerance, compensating for UV→3D roundtrip errors.
- **chore:** Complete `missing_docs` for `CommonBlock`, `FaceInfo`, and `PaveBlock` methods; fix `suspicious_double_ref_op` and `dead_code` warnings.
- **fix(trim):** `canonical_edges_share_identity()` 现在完整比较 `SectionSegment(sc_id, v1, v2)` 三元组，不再只比较 `sc_id` 导致同一 section curve 的不同 segment 被错误视为同一条边。
- **fix(api):** `run_boolean_pipeline()` 的 `all_boundary` 空选中分支现在区分操作类型：Section 操作返回空结果而非直接返回输入 solid。
- **refactor(bopds):** `TrimmingEdgeSource::SectionCurve(id)` 升级为 `SectionSegment { curve, segment_index }`，支持 segment 级别的来源身份追踪。
- **fix(api):** 空选中分支的 passthrough 路径补全了 face provenance（之前只有空的 `ProvenanceMap::default()`）。
- **feat(trim):** 新增 `group_faces_by_shared_vertices()` 基于顶点 ID 的连通分量算法（union-find），为未来 force_rebuild 多 shell 分组预留。
- **feat(intersect):** 新增 `coplanar_face_overlap_curves()` 共面面重叠区域边界检测算法（未启用，Issue #7 待完善集成）。
- **fix(api):** passthrough 路径的 edge provenance 补全：使用真实 `TrimmingEdge` 来源（`OriginalBoundaryEdge` / `SectionSegment`），不再使用合成 ID。
- **fix(trim):** `build_trimming_loops()` 容差不再混合 `parametric_tol.max(geometric_tol)`，统一使用 `geometric_tol`，避免 UV/3D 空间容差混淆。
- **feat(trim):** 新增 `group_faces_by_shared_vertices()` 基于 Vertex ID 的 union-find 连通分量分组（为 force_rebuild 多 shell 预留）。
- **refactor(trim):** `force_rebuild` 路径统一走 `sew_shell_faces`，移除 `group_faces_by_shared_vertices()` 这种 shared-vertex 启发式分组；shell 条件收紧为**仅接受 `Closed`**，并对所有 Closed shell 强制 `validate_shell_orientation`，消除 Regular/Oriented 兜底带来的不变量漂移。
- **feat(trim):** 新增 `CanonicalRebuiltEdge::SharedBoundary(v1,v2)` 身份变体：当一条边非 section curve 且被多个 face 共享时，取代 `OpenBoundary(face,…)`，让跨 face 的缝合边在 canonical 比较中自然等价。
- **fix(trim):** `canonical_edges_share_identity()` 对 `SectionSegment` 特化为只比较 `curve` id（忽略 `segment_index`），允许同一 section curve 的不同 segment 视为同一身份，修复大面被 section curve 切开后 segment 级 component 分裂的 bug。
- **feat(trim):** 新增 `split_faces_share_component_boundary()` 邻接判定：除了 `share_orientable_edge` 与 `share_vertex` 外，跨 `original_face` 的边界边若 canonical 身份一致也视为同连通分量，修复 EE 管线分裂后的 face 孤岛。
- **fix(fclass2d):** `adjust_to_range()` 周期调整改用硬边界 `v < lo` / `v > hi`，移除原 `0.5 * period` 松弛窗口，修复 u 周期 seam 上的点（例如 `u=1.0` / `u=2.0`）被错误归为 Inside 而非 OnBoundary；新增 `periodic_seam_boundary_is_on_boundary` 回归测试。
- **feat(intersect):** 新增 `try_analytical_plane_revoluted()` 平面-旋转面解析求交快路径：UV 32×32 网格按平面方程符号变号定位交叉点，容差过滤后作为 plane-revoluted 共线交点序列返回，替代部分 mesh-based 回退。
- **fix(api):** `run_boolean_pipeline_inner()` 在 `Cut` 且 `ds.split_faces` 全部 OnBoundary 时直接返回**空 BooleanResult**（`a − a = ∅`），不再走 Fuse/Common 同款 passthrough 分支；新增 `boundary_only_cut_returns_empty_result` 回归测试锁死行为。
- **refactor(api):** 移除 `Fuse` 的 assembly 失败兜底（之前在 `Err(_)` 时返回 `[a, b]` passthrough solids），改为将装配错误原样上抛；新增 `overlapping_boxes_fuse_does_not_fallback_to_passthrough_solids` 回归测试，防止未来再退化为双 solid 透传。
- **test(examples):** `bool_verify` 增加 `run_with_timeout` 超时保护（避免挂死阻塞整批验证），补齐 `cyl_cyl_fuse` 等曲面-曲面 fuse case，并保留 `_sphere_solid()` 预留供后续球面验证。
- **test(examples):** 新增 `bool_occt_verify.rs`（未跟踪时已存在），用于与 OCCT 同 case 的布尔结果旁证。
- **docs:** 同步更新 `docs/truck-bop-architecture.md` 的 trim 管线图与边身份说明；刷新 `docs/diagrams/trim-data-model.svg` 与 `trim-pipeline.svg`。
- **docs(plans):** 新增 `docs/plans/2026-04-10-current-boolean-ops-architecture.excalidraw`（当前布尔管线架构快照）与 `docs/plans/2026-04-13-truck-bop-next-steps.md`（后续开发计划）。

### Other

- Fix non intersect bboxes has invalid intersects.
- Fix syntax error when reading binary STL > 8192b.
- Fillet for a single edge.
- Write assembly to step file.
- In the README, we clarified that the subtitle is the origin of the name “truck,” and changed all instances of the term in the main text to `truck`.
- Get more precise part attributions from `Product` and `NextAssemblyUsageOccurrence`.
- Add the variable `division` to `truck_modeling::builder::rsweep`.
- Renew DAG structure.
- Fix spell and replace `Fn` to `FnMut`.
- Read assembly from step file.
- Implement assembly structure handler `truck-assembly`.
- Downgrade `cargo` for `cargo doc`. cf: https://github.com/rust-lang/rust/issues/148431
- Update docker container and `Makefile.toml` for `gpu-test`.
- Update docker container `gpu-test`
- Fix step output of `CylindricalSurface`.
- Remove `Arc` from the members of `DeviceHandler`.
- Implement `border_wires` for `Face`.
- Implement `From` and `ToSameGeometry` from `ExtrudeCurve<Line<Point3>, Vector3>` to `Plane`.
- Fix comparative phrasing.
- Fix `SceneInfo` in `polygon.wgsl`.
- Upgrade wgpu v26.
- Approximation of `RbfSurface` by `ApproxFilletSurface`.
- Align mesh aspects of general surfaces tessellation.
- Refactoring: `intersection_curve` and `Homogeneous`.
- Implement `CurveDers` and `SurfaceDers`.
- Loosened `cut_random_test` requirements.
- Higher order derivations.
- Renew `Camera`.
- Constant allocation for faster B-spline basis function.
- New implementation for `search_parameter`.
- Add `RbfSurface`.
- Add `prop_assert_near` for `proptest` integration.
- Primitives: rect, circle, and cuboid.
- The zoom of the parallel camera has been made to work.
- Minor change.
- Fix some typos.
- Saving memory of `put_together_same_attrs`.
- Closed mesh with `robust_triangluation`.
- Implement `Transformed<Matrix4>` for `PolygonMesh`.
- Fix some step output.
- `cargo upgrade -i`
- Create `CYLINDRICAL_SURFACE` by `builder::rsweep`.
- Step output for specified revoluted surface.
- Remove `println` for debugging.
- Generalize `truck_modeling::builder` for apply step parsed geometries.
- Review of the specifications for `IntersectionCurve`.
- Fix STEP header description.
- Fix some typos.
- Implement `BSplineCurve::interpole`.
- Implement `search_intersection_parameter` between surface and curve.
- Add macros: `wire` and `shell`.
- Strict derivation and `search_parameter` of `IntersectionCurve`.
- Prototyping for fillet surface with NURBS geometry.
- Implement abstract newton method.
- Minor correction of `double_projection`.
- Update algorithm of `double_projection`.
- More improve of `truck_geotrait::algo::surface::search_parameter`.
- Simplify `truck_geotrait::algo::surface::search_parameter`.
- Add the macro `truck_topology::prelude!`.

### Latest `cargo upgrade`

2026-03-13

## v0.6

### Additional APIs

- `truck_stepio::in` has been released!
  - Parse some geometries: B-spline, NURBS, elementary geometries, and so on.
  - Parse topologies: shell and solid.
  - JS wrappers.
- Implement `robust_triangulation`, trimming meshes by `SearchNearestParameter`.
- Output meshes by vtk formats.
- Split closed edges and faces, loaded from STEP (generated by other CAD systems).
- Calculate volume and center of the gravity of `PolygonMesh`.
- Derive macros for implementing `StepLength` and `DisplayByStep`.
- `area` and `include` function for a domain with several polyline boundaries.

### Updated APIs

- Add "periodic" identifier to `ParametricCurve` and `ParametricSurface`.
- Remove the `Invertible` constraint from tessellating traits.
- Features has been set up to use each module in `truck-meshalgo` separately.
- Non-bounded parameter ranges has been supported. Updates `ParametricXXX` and `BoundedXXX`.
- Derive macros in `truck-derivers` are supported for cases with generics.
- Implement `SearchNearestParameter` for `Processor`.
- Expanded coverage of tessellation API.
  - Enabled meshing when the boundary is not closed in the parameter space.
  - Add tessellate test with ABC Dataset.
- Improve `put_together_each_attrs`.
  - Add an argument to `put_together_each_attrs` to specify the tolerance.
  - Transitive clustering instead of spatial partitioning by rounding
- Improve `Shell::face_adjacency`: Common edges are now also retrieved.

### Bug fix

- Change the precision of floating point numbers when outputting STEP files.
- Updates `SearchNearestParameter` for `RevolutedCurve`.
- Fix a bug on partial `rsweep` with a negative angle.
- Add a private function `spade_round` for fixing insert error.

### Internal Improvements

- Replace `Mutex` and `Arc` more faster and compact mem.
- Refactor and renew test for `truck_modeling::geom_impl` by `proptest`.
- Add tests for traits in `truck_modeling::topo_traits`.
- Implementation for closed surface tessellation.
- Implelment `AsRef`, `Borrow`, and `Extend` for `Wire` and `Shell`.

### Misc

- Changed some naming conventions to Rust standards.
  - Make some struct naming canonical. ex: NURBSCurve -> NurbsCurve.
  - Remove `get_` prefix from `Vertex::get_point`, `Edge::get_curve`, and `Face::get_surface`.
- Put `truck_geometry::prelude` for resolve multiple re-export.
- Tutorial for v0.6 series has been released.

## v0.5

### Additional APIs

- derive macros for geometric traits [`truck-geoderive`](truck-geoderive)
- step output of open shell, worlds including several models, and `IntersectionCurve`
- parallel iterators for topological structures
- direct tessellation of `CompressedShell` and `CompressedSolid`
- direct serialization for topological data structures.
- cubic B-spline approximation
- `builder::try_wire_homotopy`
- `Solid::cut_face_by_edge`
- `Face::edge_iter` and `Face::vertex_iter`
- `IntersectionCurve` between `Plane`s can now be converted to `Line`.
- `Camera::ray`
- `EntryMap`

### Updated APIs

- `MeshableShape::triangulation`
- the Euler operations
- `Face::cut_by_edge`
- Refactoring `Search(Nearest)Parameter`.

### Bug fix

- The orientation of the normal of `builder::try_attach_plane`.
- `Shell::singular_vertices`
- binary STL output of `PolygonMesh`

### Internal Improvements

- Data integrity check during deserialization of `KnotVec`, `BSplineCurve`, and all structs constructed by `try_new`.
- Improve meshing algorithm by parallelization.
- Intersection curve with B-spline leader.
- Implement some geometric traits for `TrimmedCurve`, `UnitHyperbola` and `UnitParabola`.
- Use Line in modeling and simplify output shape of tsweep.

### Misc

- Make `TextureFormat` of surfaces `BrgaU8norm`.
- Add an example with several boundaries.
- Updates `wgpu` to `v0.14`
- Updates `spade` to `v2`.
- Change the profile of `truck-js` and remove dependencies to `wee_alloc`.

## v0.4

- The first version of `truck-stepio` has been released! One can output shapes modeled by `truck-modeling`.
- WGSL utility `math.wgsl` has been released! One can calculate invert matrices and rotation matrices.
- The processing related to linear algebra has been isolated from `truck-base` to [`matext4cgmath`](https://crates.io/crates/matext4cgmath).
- New mesh filter `Subdivision::loop_subdivision` was implemented in `truck-meshalgo`!
- In `truck-geotrait`, the trait `ParametricCurve` is decomposed into `ParametricCurve` and `BoundedCurve`.
- The method `swap_vertex` has been added to `WireFrameInstance`.
- Geometric traits has been derived to `Box`.
- Some specified geometries has been added for STEP I/O
- Comparing `BoundingBox` by inclusion relationship.
- In order to make meshing reproducible, we decided to implement random perturbations by means of a deterministic hash function.
- Some lints has been added.

## v0.3

- Specified surface for STEP I/O and modeling revolved sphere and cone.
  - In `truck-base`, the trait `Surface` is decomposed into `ParametricSurface`, `BoundedSurface`, `IncludeCurve` and `Invertible`.
  - In `truck-geometry`, specified surface, `Plane` and `Sphere`, and some decorators are prepared.
- STL handling module `stl` in `truck-polymesh`.
- In `truck-rendimpl`, wireframe for polygon.
  - Abort traits `Shape` and `Polygon`, and add new traits `IntoInstance` and `TryIntoInstance`.
- Applied wgpu v0.11 and made all shaders WGSL, including shaders for test. Now, all dependence on cmake has been removed!
  - The sample code `glsl-sandbox` becomes `wgsl-sandbox`. You can easily experience WGSL shading.
- Split `truck-base::geom_trait` into `truck-geotrait` and added some algorithms `algo`. Some methods in curves and surfaces were standardized.
- Added a new crate `truck-meshalgo`. Moved the polygon processing algorithm from polymesh to meshalgo.
- Added a new CAD meshing algorithm. Meshing trimmed surfaces. The same edge is made into the same polyline. A solid is made into a closed polygon.
- Added some meshing algorithms, including mesh collision.
- `ShapeInstance` has been removed. Tessellation should be done in advance by `truck-meshalgo` when drawing the modeled shape.
- `BSplineCurve<Point3>` was made to be `ParametricCurve3D`. Conflicts related to methods `subs` have been resolved.
- Added a new crate `truck-shapeops`, which provides solid boolean operator functions: `and` and `or`.
- Added a new crate `truck-js`, which provides wasm bindings of CAD APIs. (not released to crates.io)

## v0.2

### v0.2.1

- a small behavior change: [`NormalFilters::add_smooth_normals`](https://docs.rs/truck-polymesh/0.2.1/truck_polymesh/prelude/trait.NormalFilters.html#tymethod.add_smooth_normals).
- fix a bug: [`Splitting::into_components`](https://docs.rs/truck-polymesh/0.2.1/truck_polymesh/prelude/trait.Splitting.html#tymethod.into_components).
- an internal change: [`RenderID::gen`](https://docs.rs/truck-platform/0.2.1/truck_platform/struct.RenderID.html#method.gen).

### v0.2.0

- made `truck-polymesh` stable (well-tested and safety)
  - The member variables of [`PolygonMesh`](https://docs.rs/truck-polymesh/0.2.0/truck_polymesh/struct.PolygonMesh.html) becomes private.  
    - Destructive changes to the mesh are provided by [`PolygonMeshEditor`](https://docs.rs/truck-polymesh/0.2.0/truck_polymesh/polygon_mesh/struct.PolygonMeshEditor.html), which checks the regularity of the mesh at dropped time.
  - Mesh handling algorithms are now a public API.
    - The hidden structure `MeshHandler` was abolished and algorithms are managed as traits.
    - You can use them by importing [`truck_polymesh::prelude::*`](https://docs.rs/truck-polymesh/0.2.0/truck_polymesh/prelude/index.html).
- improved `truck-rendimpl` for higher performance and better usability
  - Wire frame rendering for shapes are now available.
    - One can create [`WireFrameInstance`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.WireFrameInstance.html) by [`InstanceCreator::create_wire_frame_instance`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.InstanceCreator.html#method.create_wire_frame_instance).
    - Try to run `cargo run --example wireframe`.
  - [`InstanceDescriptor`](https://docs.rs/truck-rendimpl/0.1.5/truck_rendimpl/struct.InstanceDescriptor.html) is separated into [`PolygonInstanceDescriptor`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.PolygonInstanceDescriptor.html) and [`ShapeInstanceDescriptor`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.ShapeInstanceDescriptor.html).
    - One can specify the precision of meshing faces by `ShapeInstanceDescriptor::mesh_precision`.
    - The old `InstanceDescriptor` is renamed to [`InstanceState`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.InstanceState.html).
    - The descriptor for wire frames is [`WireFrameInstanceDescriptor`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.WireFrameInstanceDescriptor.html).
  - added [`InstanceCreator`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.InstanceCreator.html) for generating instances.
    - `InstanceCreator` has pre-compiled shader modules as member variables.
    - [`CreateInstance`](https://docs.rs/truck-rendimpl/0.1.5/truck_rendimpl/trait.CreateInstance.html) for `Scene` is abolished.
    - `InstanceCreator` is created by [`Scene::instance_creator`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/trait.CreatorCreator.html#tymethod.instance_creator).
  - Face-wise rendering of shape is abolished.
    - Now, `ShapeInstance` is one `Rendered` struct.
    - [`RenderFace`](https://docs.rs/truck-rendimpl/0.1.5/truck_rendimpl/struct.RenderFace.html) was abolished.
  - abolished implementations `Clone` for `*Instance`. Use `*Instance::clone_instance`.
  - The texture of `InstanceState` was changed `wgpu::Texture` from `image::DynamicImage`.  
  One can generate `Texture` from `DynamicImage` by [`InstanceCreator::create_texture`](https://docs.rs/truck-rendimpl/0.2.0/truck_rendimpl/struct.InstanceCreator.html#method.create_texture).
- added inherit methods of `truck_geometry::NURBSSurface` from `BSplineSurface`.
- added a feature `serde` to `cgmath` at `truck-base`.
  - remove the explicit dependency to `cgmath` from `truck-polymesh`.
  - plans to add `nalgebra` as an alternative backend (unreleased in this version).
- abolished [`truck_platform::RenderID::default`](https://docs.rs/truck-platform/0.1.0/truck_platform/struct.RenderID.html#impl-Default) and added [`RenderID::gen`](https://docs.rs/truck-platform/0.2.0/truck_platform/struct.RenderID.html#method.gen).
- added [`Error`](https://docs.rs/truck-modeling/0.2.1/truck_modeling/errors/enum.Error.html) to `truck_modeling`.
- made [`truck_topology::CompressedShell`](https://docs.rs/truck-topology/0.2.0/truck_topology/struct.CompressedShell.html) public API and added [`truck_topology::CompressedSolid`](https://docs.rs/truck-topology/0.2.0/truck_topology/struct.CompressedSolid.html).

## v0.1

### v0.1.5

- changed a behavior of [`truck_topology::try_add_boundary`](https://docs.rs/truck-topology/0.1.1/truck_topology/struct.Face.html#method.try_add_boundary) and [`truck_topology::add_boundary`](https://docs.rs/truck-topology/0.1.1/truck_topology/struct.Face.html#method.add_boundary).
  - flip the boundary over when adding a boundary to a face with a flipped orientation
  - renew the id of the face which was added boundary

### v0.1.4

- add a method: `truck_rendimpl::*Instance::clone_instance`
- `Clone::clone for *Instance` is deprecated, and will be abolished in v0.2.

### v0.1.3

- fixed two bugs
  - [`truck_modeling::builder::homotopy`](https://docs.rs/truck-modeling/0.1.3/truck_modeling/builder/fn.homotopy.html), the vertices were in the wrong order.
  - [`truck_modeling::Mapped for Shell`](https://docs.rs/truck-modeling/0.1.3/truck_modeling/topo_traits/trait.Mapped.html#impl-Mapped%3CP%2C%20C%2C%20S%3E-for-Shell%3CP%2C%20C%2C%20S%3E), the orientation of surface was wrong.

### v0.1.2

- fixed a bug: [`truck_modeling::builder::try_attach_plane`](https://docs.rs/truck-modeling/0.1.2/truck_modeling/builder/fn.try_attach_plane.html), the orientation of plane was incorrect.

### v0.1.1

- fixed a bug: [`truck_modeling::builder::rsweep`](https://docs.rs/truck-modeling/0.1.1/truck_modeling/builder/fn.rsweep.html), the boundary was incorrect.

### v0.1.0

- first version
