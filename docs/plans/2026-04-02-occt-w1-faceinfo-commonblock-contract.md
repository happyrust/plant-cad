# 2026-04-02 | W1.1 数据结构接口草案（FaceInfo / CommonBlock）

> 目的：在不改动现网行为的前提下，为下一步实现提供可复用的、可审核的字段与方法契约。

## 1. 现状核对（已验证）

- `truck-bop/src/bopds/face_info.rs` 已有：
  - `on_vertices / in_vertices / sc_vertices`
  - `on_pave_blocks / in_pave_blocks / sc_pave_blocks`
  - `push_*` API 与去重逻辑
- `truck-bop/src/bopds/common_block.rs` 已有：
  - `pave_blocks / faces / representative_edge`
  - `new / push_pave_block / push_face` 与去重逻辑

当前可看作“基础字段已到位”，但在接口语义上仍缺：
- 统一的字段语义（字段属于哪类布尔阶段）
- 查询一致性（是否 empty、是否有效、分类视图）
- 变更日志（便于 scheduler 做 diff 快照）

---

## 2. FaceInfo 契约草案（v0.1）

### 2.1 数据字段

保留现有字段（兼容）并增加轻量只读快照能力：

```rust
pub struct FaceInfo {
    pub on_vertices: Vec<VertexId>,
    pub in_vertices: Vec<VertexId>,
    pub sc_vertices: Vec<VertexId>,
    pub on_pave_blocks: Vec<PaveBlockId>,
    pub in_pave_blocks: Vec<PaveBlockId>,
    pub sc_pave_blocks: Vec<PaveBlockId>,
    pub version: u64, // 每次更新递增，支持 scheduler 快照差异判断
    pub is_dirty: bool, // 当新增事实后置为 true；被查询后清空为 false
}
```

### 2.2 行为 API（建议新增）

1. `pub fn clear(&mut self)`
   - 清空 6 个语义列表 + 重置 `is_dirty=false`。
2. `pub fn with_version(version: u64) -> Self`
   - 可选构造，保持测试可控。
3. `pub fn on_count / in_count / sc_count -> usize`
   - 快速断言与汇总。
4. `pub fn mark_dirty(&mut self)`
   - 用于所有 push/update 之后统一调用。
5. `pub fn touched(&self) -> bool`
   - 返回 `is_dirty`，用于上报。
6. `pub fn mark_clean(&mut self)`
   - 将 `is_dirty` 置 false。

### 2.3 查询 API（建议新增）

1. `pub fn contains_vertex(&self, v: VertexId) -> bool`
2. `pub fn contains_pave_block(&self, pb: PaveBlockId) -> bool`
3. `pub fn vertex_classes(&self) -> FaceVertexClasses`
   - 返回 `enum` 快照视图（on/in/sc）

### 2.4 推演价值

- 不直接引入复杂布尔语义枚举（如 `Inside/Outside/On`）
- 先用 `on/in/sc` 与现网命名兼容
- 后续迁移 scheduler 时可直接基于 `version/touched` 做最小差分更新

---

## 3. CommonBlock 契约草案（v0.1）

### 3.1 数据字段

在当前基础上补充以下信息：

```rust
pub struct CommonBlock {
    pub pave_blocks: Vec<PaveBlockId>,
    pub faces: Vec<FaceId>,
    pub representative_edge: Option<EdgeId>,
    pub is_active: bool,
    pub extents: Option<(f64, f64)>, // 可选保留：该共享段在参数空间上的覆盖区间
    pub witness_edges: Vec<EdgeId>,
    pub witness_faces: Vec<FaceId>,
}
```

### 3.2 行为 API（建议新增）

1. `pub fn is_empty(&self) -> bool`
2. `pub fn is_effective(&self) -> bool`
   - 至少有 1 个 `pave_block` 且 `is_active`
3. `pub fn register_witness_edge(&mut self, edge: EdgeId)`
4. `pub fn register_witness_face(&mut self, face: FaceId)`
5. `pub fn has_common_between(edge_or_face: Either<EdgeId, FaceId>) -> bool`
   - 查询是否包含某几何实体（后续可返回 bool 或 enum）

### 3.3 版本化（可选）

- 引入 `version: u64`（同 FaceInfo）可支持 scheduler 做幂等对账。

---

## 4. 与 `BopDs` 对齐建议

在 `truck-bop/src/bopds/mod.rs` 中配套新增：

- `face_infos` 的 `add_if_absent`（不存在时 `default + version`）
- `register_face_info_update(face_id, |info| { ... })`
- `common_blocks` 的 `for_face(face_id) -> impl Iterator<Item=&CommonBlock>`
- `mark_face_info_snapshot(face_id) -> u64`（返回当前 version）

---

## 5. 最低实现顺序（下一步可执行）

1. 仅新增字段与 API，不动旧行为。
2. 补充 3~5 个小测试（只测 API 行为，避免几何语义）。
3. 让现有测试保持绿色。
4. 再对接到 PaveFiller 之前，不在 pipeline 中引入新调用。

---

## 6. 本草案与上级清单关系

- 上级总计划：`docs/plans/2026-04-01-occt-bool-migration-master-plan-truck-new.md`
- 执行清单：`docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md`
- 本文件对应：W1.1 / W1-2 的 API 约束输出。
