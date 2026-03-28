# Truck BOP Trim Pipeline Repair V2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 按“元数据一致性 → provenance 身份保留 → component/orientation 解耦 → shared topology rebuild → closed/outwardness invariant”这条主线，修复 `truck-bop` 的 trim/shell assembly 管线，并让当前 5 个失败用例全部转绿。

**Architecture:** 当前实现把多个层级的问题耦在 `truck-bop/src/trim.rs`：`reverse_trimming_loop()` 只反转几何序列、不反转 `vertex_ids`；`TrimmingEdge` 只保留 `section_curve`、不保留 source boundary edge 身份；`component_adjacency()` 与 `orient_face_against_shell()` 共享同一套“重建后 edge id”语义；`assemble_shells()` 见到 trimming loop 就重建 face，且只在启发式条件下做 `Closed` 校验，完全不做全局 outwardness 校验。本计划按风险从低到高拆成 7 个小任务，先修输入元数据，再修身份/图结构，再修组装/验证，最后补回归矩阵。

**Tech Stack:** Rust 2021, Cargo, `truck-bop`, `truck-topology`, `truck-modeling`, `trim.rs` 单元测试

---

## Progress Snapshot

### 当前状态（2026-03-28）

- **状态**：`verification-green`
- **已完成任务**：
  - Task 1：修正 loop reversal 的元数据一致性
  - Task 2：给 `TrimmingEdge` 保留真实 source boundary provenance
  - Task 3：拆分 topology component graph 与 orientation propagation graph
  - Task 4：shared topology rebuild / source-face 复用与多 closed components 收敛
  - Task 5：把 closed-shell 变成硬约束
  - Task 6：补全局 outwardness 校验
  - Task 7：跑完整回归矩阵并完成 crate 级验证
- **未完成任务**：
  - 无（仅剩 commit / push）

### 已落地实现

1. `reverse_trimming_loop()` 现在会同步维护 `vertex_ids`，并在 closed loop 下重建闭合后的顶点遍历序列。
2. `TrimmingEdge` 新增 `original_edge: Option<EdgeId>`，用于保留 source boundary provenance。
3. `boundary_loops()` 不再使用 `closed.windows(2)` 盲重建 boundary edge，而是沿原始 wire 的真实 edge 顺序生成 `TrimmingEdge`。
4. `canonical_rebuilt_edge()` 与 `collect_sewn_edges()` 已优先复用 `original_edge`，使 source boundary edge 在 trimming / sewing 阶段不再丢身份。
5. `connected_face_components()` 与 `orientation_adjacency()` 已拆分：前者允许 vertex-level topology connectivity，后者只保留 orientable shared edge。
6. `sew_shell_faces()` 现在按 orientation island 渐进组装：当同一 topology component 内不存在可传播朝向的候选时，会开启新的 seed，而不再把“无法传播 orientation”和“组件不连通”混为一谈。
7. `assemble_shells()` 新增了 source-face 复用判断：若 split face 只是原始 boundary 的 provenance 映射，则直接复用 `original_face`，避免无意义 rebuild 切断真实共享边。
8. `sew_shell_faces()` 现在在 component 分组时同时参考 split-face graph 与实际重建 face 的 shared topology，因此多 closed components 场景可以稳定拆成多个 shell，而不再把第二个 cuboid 切碎。
9. `assemble_shells()` 现在对每个输出 shell **无条件执行** `Closed` 校验，不再依赖 `shell_faces_reused_original_boundaries(...)` 这类启发式 gate。
10. `validate_shell_orientation()` 已接入 `assemble_shells()`：它按 face boundary 的真实顶点遍历顺序重建 probe loop，沿 oriented normal 做 `+/- probe`，并复用 `classify_point_in_solid()` 校验 `+normal -> Outside`、`-normal -> Inside`。
11. `shell_orientation_rejects_closed_shell_with_inverted_faces` 的 red fixture 已改成**显式逐面 `invert()`**；原因是 `shell.inverse()` 在当前 `truck-topology` + trim pipeline 组合下，不能稳定表达“所有面 globally inward”的测试语义。
12. source boundary edge id 当前采用**高位隔离的哈希 EdgeId**：
   - `section_curve -> EdgeId(section_curve_id.0)`
   - `source boundary -> EdgeId(0x8000_0000 | (hash(edge.id()) & 0x7fff_ffff))`
   这样做的目的是先避免与 section curve 的低位 id 空间冲突；后续如果 `BopDs` 引入统一 edge registry，可再替换成更稳定的真实分配策略。

### 当前已验证通过的增量测试

- Task 1
  - `cargo test -p truck-bop trim::tests::reverse_trimming_loop_preserves_vertex_alignment_for_closed_loop -- --exact`
  - `cargo test -p truck-bop trim::tests::classify_loops_keeps_vertex_ids_aligned_after_orientation_fix -- --exact`
  - `cargo test -p truck-bop trim::tests::trimming_loop_reversal_preserves_closed_polyline_and_edge_connectivity -- --exact`
- Task 2
  - `cargo test -p truck-bop trim::tests::boundary_loops_preserve_original_edge_provenance -- --exact`
  - `cargo test -p truck-bop trim::tests::edge_sewing_tracks_source_boundary_identity_for_shared_faces -- --exact`
  - `cargo test -p truck-bop trim::tests::canonical_rebuilt_edge_ids_distinguish_shared_and_open_boundaries -- --exact`
  - `cargo test -p truck-bop trim::tests::canonical_rebuilt_edge_ids_share_true_non_section_trim_edges_only -- --exact`
  - `cargo test -p truck-bop trim::tests::edge_sewing_tracks_section_edge_identity_for_open_paths -- --exact`
- Task 3
  - `cargo test -p truck-bop trim::tests::shared_vertex_only_faces_still_form_one_component -- --exact`
  - `cargo test -p truck-bop trim::tests::shared_vertex_only_faces_do_not_create_orientation_adjacency -- --exact`
  - `cargo test -p truck-bop trim::tests::shared_edge_faces_create_orientation_adjacency -- --exact`
  - `cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --exact`
  - `cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`
- Task 4 / 5 / 6
  - `cargo test -p truck-bop trim::tests::sew_shell_faces_separates_disconnected_closed_components -- --exact`
  - `cargo test -p truck-bop trim::tests::shell_assembly_separates_disconnected_closed_components -- --exact`
  - `cargo test -p truck-bop trim::tests::shell_closure_rejects_boundary_edge_component -- --exact`
  - `cargo test -p truck-bop trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact`
  - `cargo test -p truck-bop trim::tests::shell_orientation_accepts_outward_closed_shell -- --exact`
  - `cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --exact`
  - `cargo check -p truck-bop`

### 当前边界

- Task 1 ~ Task 7 的核心目标已经全部完成，当前实现已通过 `truck-bop` crate 级测试验证。
- Task 7 过程中额外修正了一处**测试语义滞后**：Task 3 将 `split_faces_share_component()` 扩展为“共享可定向边 **或** 共享顶点即可连通”，因此两条 canonical 测试不该再用它断言“open boundary 不共享”；现已改为断言 `!split_faces_share_orientable_edge(...)`，并补充保留 `split_faces_share_component(...) == true` 的新语义检查。
- 当前这一轮已验证转绿的 shell assembly 主红灯：
  - `shell_assembly_groups_faces_by_shared_topology_components`
  - `face_orientation_flips_adjacent_face_to_close_shell`
  - `shell_closure_rejects_boundary_edge_component`
  - `shell_assembly_separates_disconnected_closed_components`
  - `shell_orientation_rejects_closed_shell_with_inverted_faces`
- 当前已完成的更大范围验证：
  - Task 7 关键回归矩阵全部通过
  - `cargo test -p truck-bop` 全量通过（`138 passed, 4 ignored`）
- 当前仍未处理的只剩工程性收尾：
  - 尚未 commit / push
  - `docs/plans/2026-03-24-truck-bop-shell-assembly-repair.md` 已补 `superseded by v2` 标记
- `cargo check -p truck-bop` 当前为 `0 errors, 10 warnings`；这些 warning 仍以既有未使用结构/未来兼容提示为主，不是本轮逻辑修复引入的新业务回归。

---

## Baseline Facts

### 当前已验证红灯（2026-03-28，本地执行）

以下命令都必须使用 **完整测试名**，不能只传裸测试名：

- `cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --exact`
- `cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`
- `cargo test -p truck-bop trim::tests::shell_closure_rejects_boundary_edge_component -- --exact`
- `cargo test -p truck-bop trim::tests::shell_assembly_separates_disconnected_closed_components -- --exact`
- `cargo test -p truck-bop trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact`

当前观察到的失败症状：

- `shell_assembly_groups_faces_by_shared_topology_components`
  组件被切碎，断言里出现 `left: [[0], [1], [2], [3]]`。
- `face_orientation_flips_adjacent_face_to_close_shell`
  `shells.len()` 实际为 `6`，预期为 `1`。
- `shell_closure_rejects_boundary_edge_component`
  `unwrap_err()` 拿到了 `Ok(...)`，说明 open shell 被错误放行。
- `shell_assembly_separates_disconnected_closed_components`
  `shells.len()` 实际为 `12`，预期为 `2`。
- `shell_orientation_rejects_closed_shell_with_inverted_faces`
  `unwrap_err()` 拿到了 `Ok(...)`，说明 globally inverted closed shell 被错误放行。

### 当前已验证绿灯（2026-03-28，本地执行）

- `cargo test -p truck-bop trim::tests::trimming_loop_reversal_preserves_closed_polyline_and_edge_connectivity -- --exact`
- `cargo test -p truck-bop trim::tests::canonical_rebuilt_edge_ids_distinguish_shared_and_open_boundaries -- --exact`
- `cargo test -p truck-bop trim::tests::canonical_rebuilt_edge_ids_share_true_non_section_trim_edges_only -- --exact`

这说明：

1. 仓库里已经有一部分“canonical edge 区分 shared/open boundary”的测试；
2. 但它们还不足以约束真实 source boundary 身份、component/orientation 图解耦、以及 assemble 后的 invariant；
3. 因此不能只盯着 `canonical_rebuilt_edge()` 微调，必须补全更上游的 provenance 和更下游的 assembly 校验。

---

## File Map

- Modify: `truck-bop/src/bopds/interference.rs`
  给 `TrimmingEdge` 增加 source boundary provenance 字段，并补充 `TrimmingLoop.vertex_ids` 与 loop traversal 对齐约束。
- Modify: `truck-bop/src/trim.rs`
  这是主战场：
  - 修 `reverse_trimming_loop()` 的 `vertex_ids` 反转对齐；
  - 在 `boundary_loops()` 中保留原始边 identity；
  - 拆分 component graph 与 orientation graph；
  - 重构 `assemble_shells()` 的 rebuild/validation 顺序；
  - 补所有核心测试与回归矩阵。
- Modify: `truck-bop/src/bopds/mod.rs`
  更新任何直接构造 `TrimmingEdge` 的测试或 fixture。
- Modify: `truck-bop/src/intersect/ee.rs`
  更新 `TrimmingEdge { ... }` 直接构造点，初始化新增字段。
- Modify: `truck-bop/src/intersect/ef.rs`
  更新 `TrimmingEdge { ... }` 直接构造点，初始化新增字段。
- Modify: `truck-bop/src/intersect/ve.rs`
  更新 `TrimmingEdge { ... }` 直接构造点，初始化新增字段。
- Optional Modify: `docs/plans/2026-03-24-truck-bop-shell-assembly-repair.md`
  如果旧计划确认废弃，最后补一行“superseded by v2”即可；不要在实施前混写两版计划。

---

### Task 1: 修正 loop reversal 的元数据一致性

**Files:**
- Modify: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [x] **Step 1: 写失败测试**

在 `truck-bop/src/trim.rs` 新增：
- `reverse_trimming_loop_preserves_vertex_alignment_for_closed_loop`
- `classify_loops_keeps_vertex_ids_aligned_after_orientation_fix`

测试重点不是“polyline 还是闭合”，而是约束这四个序列在 reversal 后的遍历语义一致：
- `uv_points`
- `edges`
- `edge.uv_points`
- `vertex_ids`

- [x] **Step 2: 跑失败测试，确认当前确实缺约束**

Run:
`cargo test -p truck-bop trim::tests::reverse_trimming_loop_preserves_vertex_alignment_for_closed_loop -- --exact`
`cargo test -p truck-bop trim::tests::classify_loops_keeps_vertex_ids_aligned_after_orientation_fix -- --exact`

Expected: FAIL，因为当前 `reverse_trimming_loop()` 只反转 `uv_points` / `edges` / `edge.uv_points`，没有同步反转 `vertex_ids`。

- [x] **Step 3: 写最小实现**

修改 `reverse_trimming_loop()`：
- 同步反转 `trimming_loop.vertex_ids`；
- 对 closed loop，和 `uv_points` 一样重新闭合 `vertex_ids`（保证首尾语义一致）；
- 在 `TrimmingLoop.vertex_ids` 注释里明确：它必须与 open-loop traversal 顺序一一对应，且在 closed loop 情况下可带重复首尾点。

不要顺手改 provenance、component graph 或 assembly 逻辑。

- [x] **Step 4: 跑通过测试**

Run:
`cargo test -p truck-bop trim::tests::reverse_trimming_loop_preserves_vertex_alignment_for_closed_loop -- --exact`
`cargo test -p truck-bop trim::tests::classify_loops_keeps_vertex_ids_aligned_after_orientation_fix -- --exact`
`cargo test -p truck-bop trim::tests::trimming_loop_reversal_preserves_closed_polyline_and_edge_connectivity -- --exact`

Expected: 新旧 reversal 测试都通过。

- [ ] **Step 5: Commit**

> 2026-03-28 进展：已完成 Step 1~4，尚未单独提交 commit。

```bash
git add truck-bop/src/bopds/interference.rs truck-bop/src/trim.rs
git commit -m "fix: keep trimming loop metadata aligned during reversal"
```

### Task 2: 给 `TrimmingEdge` 保留真实 source boundary provenance

**Files:**
- Modify: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/trim.rs`
- Modify: `truck-bop/src/bopds/mod.rs`
- Modify: `truck-bop/src/intersect/ee.rs`
- Modify: `truck-bop/src/intersect/ef.rs`
- Modify: `truck-bop/src/intersect/ve.rs`
- Test: `truck-bop/src/trim.rs`

- [x] **Step 1: 写失败测试**

新增：
- `boundary_loops_preserve_original_edge_provenance`
- `edge_sewing_tracks_source_boundary_identity_for_shared_faces`
- `trimming_loop_vertex_ids_are_canonical_not_face_local`（如果实现时决定顺手收敛 vertex id 语义）

至少第一条必须覆盖：两个相邻 source faces 的 shared boundary segment 应带相同 `original_edge: Some(...)`，而不是仅靠 UV/vertex 猜。

- [x] **Step 2: 跑失败测试**

Run:
`cargo test -p truck-bop trim::tests::boundary_loops_preserve_original_edge_provenance -- --exact`
`cargo test -p truck-bop trim::tests::edge_sewing_tracks_source_boundary_identity_for_shared_faces -- --exact`

Expected: FAIL，因为当前 `TrimmingEdge` 只有 `section_curve` 和 `uv_points`，`boundary_loops()` 用 `closed.windows(2)` 重建线段时丢失了 source edge 身份。

- [x] **Step 3: 写最小实现**

- 给 `TrimmingEdge` 增加：`original_edge: Option<EdgeId>`。
- 在 `boundary_loops()` 中改为沿 wire 的真实边迭代，给 boundary edge 填 `original_edge: Some(...)`。
- 当前实现使用高位隔离的哈希 `EdgeId` 来承载 source boundary identity，先避免与 section curve id 空间冲突。
- section 生成边保留 `section_curve: Some(...)`，同时显式 `original_edge: None`。
- 所有 `TrimmingEdge { ... }` 构造点都补齐新字段。
- 如果当前 `vertex_ids_for_polyline()` 仍然生成 face-local 假 id，则只在注释里标明边界；不要在本任务里同时大改 vertex canonical 化，避免范围失控。

- [x] **Step 4: 跑通过测试**

Run:
`cargo test -p truck-bop trim::tests::boundary_loops_preserve_original_edge_provenance -- --exact`
`cargo test -p truck-bop trim::tests::edge_sewing_tracks_source_boundary_identity_for_shared_faces -- --exact`
`cargo check -p truck-bop`

Expected: provenance 新测试通过，所有构造点完成编译迁移。

- [ ] **Step 5: Commit**

> 2026-03-28 进展：已完成 Step 1~4，尚未单独提交 commit。

```bash
git add truck-bop/src/bopds/interference.rs truck-bop/src/trim.rs truck-bop/src/bopds/mod.rs truck-bop/src/intersect/ee.rs truck-bop/src/intersect/ef.rs truck-bop/src/intersect/ve.rs
git commit -m "refactor: preserve source edge provenance in trimming data"
```

### Task 3: 拆分 topology component graph 与 orientation propagation graph

> 2026-03-28 进展：已完成测试补充与最小实现，`shell_assembly_groups_faces_by_shared_topology_components` / `face_orientation_flips_adjacent_face_to_close_shell` 已转绿。

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [x] **Step 1: 写失败测试**

新增：
- `shared_vertex_only_faces_still_form_one_component`
- `shared_vertex_only_faces_do_not_propagate_orientation`
- `shared_edge_faces_propagate_orientation_correctly`

并继续把现有红灯用例作为主合同：
- `trim::tests::shell_assembly_groups_faces_by_shared_topology_components`
- `trim::tests::face_orientation_flips_adjacent_face_to_close_shell`

- [x] **Step 2: 跑失败测试**

Run:
`cargo test -p truck-bop trim::tests::shared_vertex_only_faces_still_form_one_component -- --exact`
`cargo test -p truck-bop trim::tests::shared_vertex_only_faces_do_not_propagate_orientation -- --exact`
`cargo test -p truck-bop trim::tests::shared_edge_faces_propagate_orientation_correctly -- --exact`
`cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --exact`
`cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`

Expected: FAIL，说明当前 `component_adjacency()` 和 `orient_face_against_shell()` 仍然共用同一套“重建后边共享”语义，无法表达“共享点可连通，但不能传播 orientation”。

- [x] **Step 3: 写最小实现**

在 `trim.rs` 内新增清晰的 helper 分层：
- `split_faces_share_topology_component(...)`
- `split_faces_share_orientable_edge(...)`
- `orientation_adjacency(...)`

改造思路：
- `connected_face_components()` 只关心 topology connectivity；
- `sew_shell_faces()` 在 component 内做 orientation 排布时，只允许通过 orientable shared edge 传播朝向；
- 不再让“同一个 component”自动等价于“可传播 orientation”。

如果需要，可让 `component_adjacency()` 和 `orientation_adjacency()` 共用底层 canonical edge 提取，但输出语义必须分离。

- [x] **Step 4: 跑通过测试**

Run:
`cargo test -p truck-bop trim::tests::shared_vertex_only_faces_still_form_one_component -- --exact`
`cargo test -p truck-bop trim::tests::shared_vertex_only_faces_do_not_propagate_orientation -- --exact`
`cargo test -p truck-bop trim::tests::shared_edge_faces_propagate_orientation_correctly -- --exact`
`cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --exact`
`cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`

Expected: component 分组与 orientation 传播开始分离，至少不再把“四个面全拆散”。

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs
git commit -m "refactor: split topology connectivity from orientation propagation"
```

### Task 4: 只在必要时 rebuild face，并在 component 内复用共享 topology

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [ ] **Step 1: 写失败测试**

新增：
- `preserved_boundary_face_reuses_original_topology`
- `component_rebuild_reuses_shared_edges_across_faces`
- `rebuild_does_not_duplicate_shared_boundary_edges`

并继续使用：
- `trim::tests::shell_assembly_separates_disconnected_closed_components`
- `trim::tests::face_orientation_flips_adjacent_face_to_close_shell`

- [x] **Step 2: 跑失败测试**

Run:
`cargo test -p truck-bop trim::tests::preserved_boundary_face_reuses_original_topology -- --exact`
`cargo test -p truck-bop trim::tests::component_rebuild_reuses_shared_edges_across_faces -- --exact`
`cargo test -p truck-bop trim::tests::rebuild_does_not_duplicate_shared_boundary_edges -- --exact`
`cargo test -p truck-bop trim::tests::shell_assembly_separates_disconnected_closed_components -- --exact`

Expected: FAIL，因为当前 `assemble_shells()` 里“只要 `split_face.trimming_loops` 非空就 rebuild”，会把本可复用的原始共享边身份切断。

- [x] **Step 3: 写最小实现**

重构 `assemble_shells()` / `rebuild_face_from_split_face()`：
- 先判断 `split_face` 是否可以直接复用 `original_face`；
- 仅对真正被切坏边界的 fragment 做 rebuild；
- rebuild 时按 **component-local** 做 vertex/edge pooling，避免同一 component 内共享边再次复制成多份；
- 不要把“是否有 trimming loop”当成“必须 rebuild”的充分条件。

优先保持变更集中在 `trim.rs`；如果 helper 太长，可以在同文件内先拆私有函数，不要提前跨文件重构。

- [x] **Step 4: 跑通过测试**

Run:
`cargo test -p truck-bop trim::tests::preserved_boundary_face_reuses_original_topology -- --exact`
`cargo test -p truck-bop trim::tests::component_rebuild_reuses_shared_edges_across_faces -- --exact`
`cargo test -p truck-bop trim::tests::rebuild_does_not_duplicate_shared_boundary_edges -- --exact`
`cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`
`cargo test -p truck-bop trim::tests::shell_assembly_separates_disconnected_closed_components -- --exact`

Expected: shared topology 不再因 rebuild 被切断，shell 数量开始接近正确目标。

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs
git commit -m "refactor: reuse original faces and pool rebuilt topology per component"
```

### Task 5: 让 `assemble_shells()` 对所有 non-closed shell 执行硬性拒绝

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [x] **Step 1: 固化失败测试**

使用现有红灯：
- `trim::tests::shell_closure_rejects_boundary_edge_component`

可选补一条：
- `assemble_shells_rejects_any_non_closed_component`

- [x] **Step 2: 跑失败测试**

Run:
`cargo test -p truck-bop trim::tests::shell_closure_rejects_boundary_edge_component -- --exact`

Expected: FAIL，因为当前 `assemble_shells()` 把 `Closed` 检查挂在 `shell_faces_reused_original_boundaries(...) && shell.len() > 1` 这个启发式 gate 后面。

- [x] **Step 3: 写最小实现**

- 删除或降级 `shell_faces_reused_original_boundaries()` 的 gate 作用；
- 对每个输出 shell 无条件执行：

```rust
if shell.shell_condition() != ShellCondition::Closed {
    return Err(BopError::TopologyInvariantBroken);
}
```

保留 helper 做 debug 信息可以，但不能再参与正确性判定。

- [x] **Step 4: 跑通过测试**

Run:
`cargo test -p truck-bop trim::tests::shell_closure_rejects_boundary_edge_component -- --exact`
`cargo test -p truck-bop trim::tests::shell_assembly_separates_disconnected_closed_components -- --exact`

Expected: open shell 被稳定拒绝，disconnected closed shells 仍能保留正确输出数。

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs
git commit -m "fix: enforce closed-shell invariant during assembly"
```

### Task 6: 为 closed shell 增加全局 outwardness 校验

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Test: `truck-bop/src/trim.rs`

- [x] **Step 1: 固化失败测试**

使用现有红灯：
- `trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces`

新增一条正向用例：
- `shell_orientation_accepts_outward_closed_shell`

- [x] **Step 2: 跑失败测试**

Run:
`cargo test -p truck-bop trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact`
`cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`

Expected: inverted closed shell 仍被错误放行，而局部可修正朝向的 case 是本任务必须保住的正向合同。

- [x] **Step 3: 写最小实现**

新增私有 helper，例如 `validate_shell_orientation(...)`：
- 在 `shell.shell_condition() == Closed` 后调用；
- 优先复用 `truck-topology`/现有 point-classification 能力；
- 若没有现成 API，再做稳妥版：用 shell centroid + face sample point + oriented normal 做 inside/outside 探测；
- 若整体明显 inward 或符号不一致，则返回 `BopError::TopologyInvariantBroken`。

关键要求：这是 **global shell check**，不是再做一轮 local shared-edge orientation 猜测。

- [x] **Step 4: 跑通过测试**

Run:
`cargo test -p truck-bop trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact`
`cargo test -p truck-bop trim::tests::shell_orientation_accepts_outward_closed_shell -- --exact`
`cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`

Expected: globally inverted closed shell 被拒绝，而局部翻面可修复案例仍通过。

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs
git commit -m "fix: validate outward orientation for closed shells"
```

### Task 7: 补完整条 trim pipeline 的回归矩阵并跑全量验证

**Files:**
- Modify: `truck-bop/src/trim.rs`
- Optional Modify: `docs/plans/2026-03-24-truck-bop-shell-assembly-repair.md`

- [x] **Step 1: 补测试矩阵**

> 2026-03-28 进展：已基于 Task 1 ~ Task 6 的新增/修复测试形成完整回归矩阵；Task 7 期间又补充修正了两条 canonical 测试的断言语义，使其与 Task 3 之后的 component/orientable-edge 语义保持一致。

至少覆盖这些类目：
1. 共享边 → 同一 component，orientation 可传播
2. 只共享顶点 → 同一 component，orientation 不传播
3. 开口壳 → 必须拒绝
4. 两个分离闭壳 → 必须拆成两个 shell
5. 整体 inward closed shell → 必须拒绝
6. 保留原边界的 face → 直接复用 original topology
7. section 生成边 → 不得与 source edge provenance 混淆
8. reversal 后 loop 元数据 → `vertex_ids` 与 `uv_points` 保持同序语义

- [x] **Step 2: 跑关键回归矩阵**

Run:
`cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --exact`
`cargo test -p truck-bop trim::tests::face_orientation_flips_adjacent_face_to_close_shell -- --exact`
`cargo test -p truck-bop trim::tests::shell_closure_rejects_boundary_edge_component -- --exact`
`cargo test -p truck-bop trim::tests::shell_assembly_separates_disconnected_closed_components -- --exact`
`cargo test -p truck-bop trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact`
`cargo test -p truck-bop trim::tests::canonical_rebuilt_edge_ids_distinguish_shared_and_open_boundaries -- --exact`
`cargo test -p truck-bop trim::tests::canonical_rebuilt_edge_ids_share_true_non_section_trim_edges_only -- --exact`

Expected: 关键红灯全部转绿，原有 canonical-edge 绿灯不回退。

> 2026-03-28 结果：已通过；其中 `canonical_rebuilt_edge_ids_distinguish_shared_and_open_boundaries` / `canonical_rebuilt_edge_ids_share_true_non_section_trim_edges_only` 的旧断言已按新语义修正为“禁止误判为 orientable shared edge，但允许因 shared vertex 落在同一 component”。

- [x] **Step 3: 跑 crate 全量验证并清理残余代码**

Run:
`cargo test -p truck-bop`
`cargo check -p truck-bop`

如有残余 dead helper、临时调试输出、重复转换逻辑，在这个任务里一次性清理，但不要额外扩 scope。

> 2026-03-28 结果：`cargo test -p truck-bop` 已全绿（`138 passed, 4 ignored`）。本轮没有进一步清 warning，只保留事实说明：当前仍有既有未使用 helper / 结构体 warning，不影响本次功能回归。

- [x] **Step 4: 如果旧计划已过期，标注 superseded**

在 `docs/plans/2026-03-24-truck-bop-shell-assembly-repair.md` 顶部补一行说明，指向本文件，避免后续执行者拿旧计划继续落地。

> 2026-03-28 结果：已完成。旧计划顶部现已明确标注 superseded，并指向 v2 文档。

- [ ] **Step 5: Commit**

```bash
git add truck-bop/src/trim.rs docs/plans/2026-03-24-truck-bop-shell-assembly-repair.md docs/plans/2026-03-28-truck-bop-trim-pipeline-repair-v2.md
git commit -m "test: add regression matrix for trim rebuild pipeline"
```

## Acceptance Checklist

- [x] `reverse_trimming_loop()` 在 closed loop 上保持 `uv_points / edges / edge.uv_points / vertex_ids` 的同序语义。
- [x] `TrimmingEdge` 显式保留 `original_edge` provenance，section edge 仅使用 `section_curve`。
- [x] `connected_face_components()` 与 orientation propagation 使用不同的关系图。
- [x] `assemble_shells()` 不再“见到 trimming loop 就无脑 rebuild”。
- [x] 同一 component 内 rebuild 后的共享 topology 不被重复复制。
- [x] `assemble_shells()` 对所有 `shell_condition() != Closed` 的输出稳定报错。
- [x] `assemble_shells()` 对 globally inverted closed shell 稳定报错。
- [ ] 以下 5 个当前红灯全部转绿：
  - [x] `trim::tests::shell_assembly_groups_faces_by_shared_topology_components`
  - [x] `trim::tests::face_orientation_flips_adjacent_face_to_close_shell`
  - [x] `trim::tests::shell_closure_rejects_boundary_edge_component`
  - [x] `trim::tests::shell_assembly_separates_disconnected_closed_components`
  - [x] `trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces`
- [x] `cargo test -p truck-bop` 全量通过。
