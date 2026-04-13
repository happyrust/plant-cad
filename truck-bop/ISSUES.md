# truck-bop 待创建 GitHub Issues

以下 Issue 描述了代码审查中发现的剩余改进项，按优先级排列。

---

## Issue 1 — P0: `uv_to_model_point` 丢失 Z 坐标

**Title:** `[truck-bop] P0: uv_to_model_point loses Z coordinate — representative points are always on Z=0 plane`

**Labels:** `bug`

### Problem

In `truck-bop/src/trim.rs`, the function `uv_to_model_point` converts a 2D UV point to a 3D point by hardcoding `Z = 0.0`:

```rust
fn uv_to_model_point(point: Point2) -> Point3 {
    Point3::new(point.x, point.y, 0.0)
}
```

This is incorrect for any face whose surface is not on the XY plane. The function is used by:
- `loop_representative_point` — calculates the representative 3D point for classification
- `collect_fragment_vertices` — gathers vertex coordinates for merging

Because `classify_split_faces_against_operand` relies on the representative point to determine whether a split face is inside/outside the opposite operand, this bug causes **incorrect boolean operation results** for any non-XY-planar geometry.

### Proposed Fix

Replace `uv_to_model_point` with actual surface evaluation. The function chain needs access to the face's surface so it can call `surface.subs(uv.x, uv.y)` to get the correct 3D point. This requires threading a surface reference through `representative_point` → `loop_representative_point`, or storing the surface reference in `SplitFace` / `TrimmingLoop`.

### Impact

**Blocking** — all split-face classification is wrong for non-trivial geometry.

---

## Issue 2 — P0: `SPHint::from_face` 和 point-in-face 使用 XY 坐标代替 UV 参数

**Title:** `[truck-bop] P0: SPHint::from_face uses 3D XY coordinates instead of UV parameters`

**Labels:** `bug`

### Problem

In `intersect/ef.rs` and `intersect/vf.rs`, the `SPHint::from_face` function computes a search hint by averaging the **3D XY coordinates** of face boundary vertices:

```rust
fn from_face<C, S>(face: &Face<Point3, C, S>) -> Option<(f64, f64)> {
    // ...
    for vertex in boundary.vertex_iter() {
        sum_u += vertex.point().x;  // ← 3D X, not UV u
        sum_v += vertex.point().y;  // ← 3D Y, not UV v
    }
}
```

Similarly, `point_on_or_inside_wire` projects 3D vertices to `Point2` via `(x, y)` instead of computing actual UV parameters. This means:
- Search hints are wrong for non-planar surfaces
- Point-in-face tests operate in the wrong coordinate space

### Proposed Fix

Use `surface.search_parameter(vertex.point(), ...)` to obtain actual UV coordinates for each boundary vertex, then compute the hint centroid and polygon in true parameter space.

### Impact

**Blocking** — intersection detection is incorrect for curved surfaces.

---

## Issue 3 — ~~P0: `classify_point_in_solid` 仅支持轴对齐盒体~~ ✅ RESOLVED

已实现三层分类器：AABB 快速路径 → Newton ray-casting → nearest-face 法向量回退。旋转 box 测试通过。

---

## Issue 4 — ~~P1: `vertex_ids_for_polyline` 使用算术公式生成 ID~~ ✅ RESOLVED

已改为共享单调递增计数器 `vertex_counter: &mut u32`，由 `build_trimming_loops` 从 `bopds.next_generated_vertex_id` 取出并写回。
