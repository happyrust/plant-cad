# Truck BOP PaveFiller Migration Plan

## 1. 背景与目标

目标不是逐行复刻 OCCT 的 `BOPAlgo_PaveFiller`，而是把它在布尔运算里的**核心职责**完整迁移到 `truck-bop`：

1. 把 `VE / EE / EF / FF` 的相交结果统一收敛为“边参数空间里的切分点”；
2. 基于这些切分点构建 `Pave / PaveBlock / CommonBlock / FaceInfo`；
3. 进一步生成 `SplitEdge / TrimmingLoop / SplitFace`；
4. 最终打通 `section / common / cut / fuse` 的公共前置流水线。

一句话概括：

> 把 OCCT 的 “PaveFiller + DS” 思想，迁移成 `truck-bop` 内部统一的相交事实源与切分调度器。

---

## 2. 当前现状（已验证事实）

### 2.1 `truck-new` 已有能力

当前 `truck-bop` 并不是从零开始，已经具备了迁移所需的几个关键基础：

- 已有 `Pave`
  - 文件：`truck-bop/src/bopds/pave.rs`
  - 当前字段已经包含 `edge / vertex / parameter / tolerance`
- 已有 `PaveBlock`
  - 文件：`truck-bop/src/bopds/pave_block.rs`
  - 当前能表达原始边上的一个参数区间段
- 已有 `BopDs`
  - 文件：`truck-bop/src/bopds/mod.rs`
  - 已能持有 `paves / pave_blocks / face_info / interferences`
- 已有局部相交模块
  - `truck-bop/src/intersect/ve.rs`
  - `truck-bop/src/intersect/ee.rs`
  - `truck-bop/src/intersect/ef.rs`
  - `truck-bop/src/intersect/ff.rs`
- 已有后段 trim / split-face / shell-assembly 主线
  - 文件：`truck-bop/src/trim.rs`
- 已有一批对 trim 管线很有价值的回归测试
  - 例如：
    - `shared_vertex_only_faces_still_form_one_component`
    - `shared_edge_faces_create_orientation_adjacency`
    - `shell_assembly_groups_faces_by_shared_topology_components`

### 2.2 当前缺口

与 OCCT 的 `PaveFiller + BOPDS_DS` 相比，当前 `truck-bop` 还缺以下关键层：

1. **缺统一调度器**
   - 目前相交与后处理仍然是分散的；
   - 没有一个统一的 `PaveFiller`-style 流程来串联 `VE -> EE -> EF -> FF -> split -> trim`。

2. **`FaceInfo` 只是占位**
   - 文件：`truck-bop/src/bopds/face_info.rs`
   - 目前没有 OCCT 风格的 `On / In / Sc` 三套事实池。

3. **`CommonBlock` 只是占位**
   - 文件：`truck-bop/src/bopds/common_block.rs`
   - 还不能表达“多个重合 pave block 的共享组”。

4. **`EE` 只返回参数对，没有落库**
   - 文件：`truck-bop/src/intersect/ee.rs`
   - 目前更像求交工具函数，而不是 DS 事实生产者。

5. **`EF / FF` 结果没有完整落到切分结构**
   - 已有 interference / section curve，但尚未统一进入：
     - `pave block`
     - `common block`
     - `face state`
     - `split edge`

6. **Public API 仍未接通**
   - 文件：`truck-bop/src/lib.rs`
   - `common / fuse / cut / section` 仍为 `NotImplemented`

---

## 3. 迁移原则

### 3.1 不做逐行镜像

不建议把 OCCT 的类层级、命名和所有内部表逐字照搬到 Rust。原因：

- `truck-bop` 现有代码已经有自己的模块划分；
- Rust 类型系统与 OCCT 的 Handle + mutable DS 模式不同；
- 当前最值钱的是 **“语义迁移”**，而不是 **“类结构复刻”**。

### 3.2 优先迁移职责闭环

本次迁移应优先保留以下职责闭环：

```text
VE / EE / EF / FF
  -> parameter paves
  -> pave blocks
  -> common blocks / face states
  -> split edges
  -> trimming loops / split faces
  -> section / common / cut / fuse
```

### 3.3 先补数据层，再接 public API

推荐采用“数据层先行”的方式：

1. 先把 `BopDs` 升级成真正的事实源；
2. 再引入统一 `PaveFiller` 调度器；
3. 最后接通 `section / common / cut / fuse`。

这样可以最大程度复用当前 `trim.rs` 已验证通过的后段逻辑，避免一次性推倒重来。

---

## 4. 目标架构

### 4.1 推荐架构图

```mermaid
flowchart TD
  A[输入 Shape / Shell / Solid] --> B[PaveFiller]
  B --> C[VE / EE / EF / FF 相交]
  C --> D[BopDs 事实层]
  D --> E[Pave / PaveBlock]
  D --> F[CommonBlock / FaceInfo]
  E --> G[SplitEdge]
  F --> H[TrimmingLoop / SplitFace]
  G --> H
  H --> I[Section]
  H --> J[Common]
  H --> K[Cut]
  H --> L[Fuse]
```

### 4.2 模块落点

建议增补和调整如下文件：

#### 新增

- `truck-bop/src/pave_filler.rs`

#### 修改

- `truck-bop/src/lib.rs`
- `truck-bop/src/pipeline.rs`
- `truck-bop/src/trim.rs`
- `truck-bop/src/bopds/mod.rs`
- `truck-bop/src/bopds/pave_block.rs`
- `truck-bop/src/bopds/common_block.rs`
- `truck-bop/src/bopds/face_info.rs`
- `truck-bop/src/bopds/interference.rs`
- `truck-bop/src/intersect/ve.rs`
- `truck-bop/src/intersect/ee.rs`
- `truck-bop/src/intersect/ef.rs`
- `truck-bop/src/intersect/ff.rs`

---

## 5. 开发方案（推荐方案 A）

## 5.1 方案 A：语义对齐优先，最小侵入接管（推荐）

### 核心思想

- 保留现有 `trim.rs` 主线；
- 不大改现有几何求交模块的内部求解方式；
- 新增一个 Rust 风格的 `PaveFiller`，负责：
  - 调用现有相交模块
  - 把结果写进 `BopDs`
  - 驱动 `SplitPaveBlocks / MakeSplitEdges / BuildTrimmingLoops / BuildSplitFaces`

### 优点

- 与现有代码最兼容；
- 风险最小；
- 能逐阶段落测试；
- 容易保持当前 trim 回归矩阵不倒退。

### 缺点

- 中间阶段会出现一段“新旧共存”的代码；
- `BopDs` 和 `trim.rs` 之间需要做几次接口收敛。

## 5.2 方案 B：一次性重构为 `PaveFiller + Builder` 分层（不推荐首轮）

### 核心思想

- 直接把 `truck-bop` 重构为类似 OCCT 的：
  - `PaveFiller`
  - `Builder`
  - `Section`
  - `Boolean`

### 风险

- 改动面太大；
- 当前 `trim.rs` 的稳定性会被打断；
- 难以在中间阶段保持全部测试绿灯。

### 结论

首轮不建议采用。等方案 A 跑通后，如果需要进一步整理 API，再考虑第二阶段重构。

---

## 6. Implementation Plan（分阶段开发计划）

## Phase 0：冻结事实源与基线

### 目标

先把当前可复跑基线固定下来，避免迁移过程中失去比较对象。

### 任务

- [ ] 记录当前 `truck-bop` 的 `cargo test -p truck-bop`
- [ ] 记录 `cargo check -p truck-bop`
- [ ] 记录 `cargo check --release -p truck-bop`
- [ ] 记录当前 trim 关键测试矩阵

### 基线命令

```bash
cargo test -p truck-bop
cargo check -p truck-bop
cargo check --release -p truck-bop
```

---

## Phase 1：升级 `BopDs` 为统一事实源

### 目标

把当前占位型 DS 升级成可承接 `PaveFiller` 的真实数据层。

### 任务

- [ ] 扩展 `FaceInfo`，拆分为：
  - `on_vertices`
  - `in_vertices`
  - `sc_vertices`
  - `on_pave_blocks`
  - `in_pave_blocks`
  - `sc_pave_blocks`
- [ ] 扩展 `CommonBlock`，支持：
  - 多个 `pave_block`
  - representative / canonical block
  - tolerance / semantic kind
- [ ] 在 `BopDs` 中新增：
  - `common_blocks`
  - `pave_block_to_common_block`
  - `interference_pairs`
- [ ] 规范 `edge -> paves -> pave_blocks` 的 rebuild API

### 目标结果

使 `BopDs` 能作为整个相交-切分-组装阶段的事实源，而不是临时缓存。

### 测试迁移

新增测试：

- [ ] `face_info_tracks_on_in_sc_separately`
- [ ] `common_block_can_group_multiple_pave_blocks`
- [ ] `bopds_can_bind_pave_block_to_common_block`
- [ ] `bopds_can_store_interference_pair_index`

---

## Phase 2：统一 `PaveBlock` 生命周期

### 目标

把 `PaveBlock` 从“静态参数区间”升级成“可被追加切分点、可被再次细分”的真实切分单元。

### 任务

- [ ] 在 `PaveBlock` 中加入 `ext_paves`
- [ ] 增加 `result_edge` 或等价映射，用于切分后结果边关联
- [ ] 增加 `split_pave_blocks_for_edges(...)`
- [ ] 为 micro-segment / tolerance 重合段定义 `unsplittable` 语义

### 测试迁移

- [ ] `split_pave_blocks_inserts_extra_paves_in_sorted_order`
- [ ] `split_pave_blocks_creates_n_minus_one_blocks_after_extra_paves`
- [ ] `unsplittable_micro_segment_is_preserved_but_not_split_again`

---

## Phase 3：把 `VE` 升级为后处理可消费事实

### 目标

把当前 `VE` 从“直接加一个 pave”升级成“命中 pave block -> 追加 ext pave -> split”的统一流程。

### 任务

- [ ] `VE` 命中边上对应 block，而不是只向全局 `paves` 追加
- [ ] 命中 endpoint 时复用已有端点 `pave`
- [ ] 命中新参数点时：
  - 生成 vertex
  - 追加到 block `ext_paves`
  - 触发该 edge 的 block split

### 测试迁移

- [ ] `ve_intersection_appends_extra_pave_into_matching_block`
- [ ] `ve_intersection_splits_edge_after_extra_pave_is_added`
- [ ] `ve_intersection_does_not_duplicate_existing_endpoint_pave`

---

## Phase 4：把 `EE` 升级为共享顶点 / 共享段事实

### 目标

让 `EE` 不再只是 `(edge1, edge2, t1, t2)` 的参数对函数，而是真正写回 `BopDs`。

### 任务

- [ ] 扩展 `EEInterference`
  - `VertexHit`
  - `OverlapHit`
- [ ] 对单点交：
  - 共用或新建 vertex
  - 在两条边对应 block 上追加 ext paves
- [ ] 对共线重合：
  - 生成 paired `pave blocks`
  - 建立 `CommonBlock`
- [ ] 明确 EE 与 trim 所需 shared-boundary 事实的映射关系

### 测试迁移

- [ ] `ee_perpendicular_segments_create_shared_vertex_and_split_both_edges`
- [ ] `ee_endpoint_touch_reuses_existing_vertices_without_duplicate_split`
- [ ] `ee_colinear_overlap_creates_common_block_not_single_vertex_only`

---

## Phase 5：把 `EF` 接到 `FaceInfo` 与切分闭环

### 目标

使 `EF` 不只是“边与面有交”，而是能把交点/共线段归入 face state 和 block 事实中。

### 任务

- [ ] 扩展 `EFInterference`
  - point-hit
  - overlap-hit
- [ ] point-hit：
  - 更新 edge paves
  - 写入 `FaceInfo` 顶点状态
- [ ] overlap-hit：
  - 生成或绑定 `pave block`
  - 写入 `FaceInfo.on_pave_blocks`
  - 必要时生成 `CommonBlock`

### 测试迁移

- [ ] `ef_point_intersection_updates_face_on_or_in_state`
- [ ] `ef_intersection_adds_edge_parameter_pave_for_split`
- [ ] `ef_edge_on_face_boundary_is_promoted_to_common_block`

---

## Phase 6：把 `FF` 的 section curve 升级为 section-edge block

### 目标

把 `FF` 从“求出 section curve”升级成“为 trimming / section 提供可消费的 section-edge blocks”。

### 任务

- [ ] 保留现有 `SectionCurve`
- [ ] 给 `SectionCurve` 增加 curve 上的 `paves`
- [ ] 由 curve 上相邻 `paves` 生成 section-edge `pave blocks`
- [ ] 同时登记到两个相交面的 `FaceInfo.sc_pave_blocks`
- [ ] 与 `trim.rs::build_trimming_loops()` 对接

### 测试迁移

- [ ] `ff_section_curve_projects_ef_vertices_into_curve_paves`
- [ ] `ff_section_edge_is_registered_in_both_faces_sc_pool`
- [ ] `ff_closed_loop_section_enables_trimming_loop_construction`

---

## Phase 7：新增统一 `PaveFiller` 调度器

### 目标

新增统一前置流水线，接管布尔运算的相交与切分调度。

### 任务

- [ ] 新增 `truck-bop/src/pave_filler.rs`
- [ ] 对外暴露统一入口，例如：
  - `build_interferences`
  - `fill_paves`
  - `split_pave_blocks`
  - `make_split_edges`
  - `make_blocks`
  - `build_trimming_loops`
  - `build_split_faces`
- [ ] 让 `section()`、`common()` 先接真主链
- [ ] 后续再让 `cut()`、`fuse()` 接入

### 建议执行顺序

```text
VV (可选占位)
-> VE
-> EE
-> VF
-> EF
-> FF
-> split_pave_blocks
-> make_split_edges
-> make_blocks
-> build_trimming_loops
-> build_split_faces
```

### 测试迁移

- [ ] `section_pipeline_runs_ve_ee_ef_ff_and_builds_split_edges`
- [ ] `common_pipeline_returns_not_notimplemented_for_two_boxes`

---

## Phase 8：接通 `cut / fuse / common / section`

### 目标

在不破坏现有 trim 后段的前提下，逐个接通 public API。

### 任务

- [ ] `section()`
  - 优先接通，作为最薄的一层输出
- [ ] `common()`
  - 复用 split face 分类逻辑先打通
- [ ] `cut()`
  - 基于 split face + classify + assemble 实现
- [ ] `fuse()`
  - 最后接通，处理 shared topology 合并

### 复用链路

优先复用现有：

- `build_split_faces`
- `classify_split_faces_against_operand`
- `merge_equivalent_vertices`
- `sew_fragment_edges`
- `assemble_shells`
- `build_solids_from_shells`

### 测试迁移

- [ ] `common_two_overlapping_boxes_returns_closed_solid`
- [ ] `cut_box_minus_box_returns_closed_solid_with_expected_component_count`
- [ ] `fuse_two_touching_boxes_reuses_shared_topology_without_open_shell`

---

## Phase 9：全量回归与收口

### 目标

确保迁移后的前置流水线与现有 trim 后段完整收敛，不引入结构性回退。

### 任务

- [ ] 跑新增 DS / intersect / filler 测试
- [ ] 跑既有 trim 回归矩阵
- [ ] 跑 `cargo test -p truck-bop`
- [ ] 跑 `cargo check -p truck-bop`
- [ ] 跑 `cargo check --release -p truck-bop`
- [ ] 检查 `deny(warnings)` 相关 release 行为

---

## 7. 测试迁移方案

测试迁移不建议“一次性搬 OCCT 全量 case”，而应采用三层策略。

## 7.1 第一层：保留并扩展当前 `truck-bop` 已有测试

这些测试已经贴近当前 Rust 实现，应原地保留并逐步增强：

- `pave_sorting_adds_missing_edge_endpoints`
- `paveblock_creation_creates_one_block_per_consecutive_pair`
- `ve_intersection_detects_vertex_on_edge_and_generates_pave`
- `ff_plane_plane_section_generates_section_curve`

作用：

- 保护当前已有语义；
- 为后续测试扩展提供稳定起点。

## 7.2 第二层：迁移 OCCT 语义级测试，而不是源码级测试

重点迁移的是“语义合同”，不是 OCCT 内部类接口。

建议迁移的 OCCT 语义主题：

1. **边上参数切分点**
   - 交点必须落到边参数上，而不只是 3D 点
2. **相邻 pave 构成 pave block**
   - block 数量应由排序后的切分点数量决定
3. **共线重合必须形成 common block**
   - 不能退化成多个离散点
4. **face-face 交线必须写入两个面的 section 池**
5. **split edge 必须可回溯原始 edge provenance**

这部分测试就是上面 Phase 1-8 里列出的新增测试矩阵。

## 7.3 第三层：现有 trim 回归矩阵作为总保护网

以下现有测试应保留为最终验收网：

- `boundary_loops_preserve_original_edge_provenance`
- `edge_sewing_tracks_source_boundary_identity_for_shared_faces`
- `shared_vertex_only_faces_still_form_one_component`
- `shared_edge_faces_create_orientation_adjacency`
- `shell_assembly_groups_faces_by_shared_topology_components`
- `shell_closure_rejects_boundary_edge_component`
- `shell_orientation_rejects_closed_shell_with_inverted_faces`

原因：

这些测试已经覆盖了当前 `truck-bop` 在“碎片 -> shell -> solid”阶段最容易回退的地方，是比单点求交更接近业务结果的保护网。

---

## 8. 任务清单（Task List）

### 会话 A：数据层收敛

- [ ] Phase 0
- [ ] Phase 1
- [ ] Phase 2

### 会话 B：单边与双边切分

- [ ] Phase 3
- [ ] Phase 4

### 会话 C：面侧状态与 section

- [ ] Phase 5
- [ ] Phase 6

### 会话 D：统一调度器与 public API

- [ ] Phase 7
- [ ] Phase 8
- [ ] Phase 9

---

## 9. 风险与缓解

### 风险 1：`PaveBlock` 重构影响现有 trim 主线

缓解：

- 先保证旧字段可兼容；
- 先在 DS 层加字段，不急于改 trim 消费侧；
- 逐阶段把 trim 改为消费新事实。

### 风险 2：`EE / EF / FF` 语义混用导致重复切分

缓解：

- 在 `BopDs` 中增加 interference pair 去重；
- `pave` 归一化必须始终按 `(edge, parameter)` 收敛。

### 风险 3：共线重合段难以稳定归组

缓解：

- 先把 `OverlapHit` 独立建模；
- 先支持最常见的线段/圆弧重合场景；
- 复杂曲线重合放到第二轮增强。

### 风险 4：public API 提前接通导致表面可用、内部失真

缓解：

- 按 `section -> common -> cut -> fuse` 顺序逐步接；
- 每接一个 API 都补最小闭环测试。

---

## 10. 验收标准

满足以下条件，才算首轮迁移完成：

- [ ] `BopDs` 能完整表达：
  - `Pave`
  - `PaveBlock`
  - `CommonBlock`
  - `FaceInfo(On / In / Sc)`
  - `VE / EE / EF / FF`
- [ ] 四类交叉都能统一收敛到“边参数空间切分点”
- [ ] `SplitPaveBlocks` / `MakeSplitEdges` 打通
- [ ] `FF` 生成的 section-edge block 能进入 `FaceInfo.sc_pave_blocks`
- [ ] `section / common / cut / fuse` 不再是 `NotImplemented`
- [ ] 当前 trim 回归矩阵不回退
- [ ] 以下命令全绿：

```bash
cargo test -p truck-bop
cargo check -p truck-bop
cargo check --release -p truck-bop
```

---

## 11. 最终建议

### 建议结论

推荐采用 **方案 A：语义对齐优先，最小侵入接管**。

### 原因

因为 `truck-bop` 当前已经有：

- `Pave / PaveBlock`
- `VE / EF / FF` 的基础相交结构
- `trim.rs` 的可运行后段

所以最优路径不是“推倒重写”，而是：

1. 把 `BopDs` 升级成真实事实层；
2. 把各相交模块的输出统一收敛到 `pave / pave block / common block / face info`；
3. 引入统一 `PaveFiller` 调度器；
4. 最后接通 `section / common / cut / fuse`。

### 建议执行顺序

优先顺序建议如下：

```text
FaceInfo / CommonBlock / PaveBlock 生命周期
-> VE
-> EE
-> EF
-> FF
-> PaveFiller
-> section/common
-> cut/fuse
```

这个顺序最符合当前 `truck-bop` 的现状，也最容易在每一步都留下可回归的测试证据。


## 12. Trim provenance milestone snapshot (2026-04-01)

The trim provenance refactor mission intentionally stopped short of the broader pave-filler work above and locked in a narrower migration boundary inside `truck-bop`.

### Delivered model changes

- `TrimmingEdge` now carries explicit `TrimmingEdgeProvenance` instead of the legacy paired `section_curve` / `original_edge` encoding.
- `TrimmingTopologyKey` is the shared identity model across trimming loops, sewn edges, and canonical rebuild logic.
- `SewnEdgeSource` stores topology keys directly, so section curves and generated edges are no longer coerced into lossy `EdgeId` fallbacks.

### Provenance categories to preserve in future work

- `SourceBoundary(EdgeId)`: source face boundary that should remain shareable across sibling fragments and shell assembly.
- `SectionCurve(SectionCurveId)`: FF-derived trimming identity that must stay stable across both owning faces and sewn open paths.
- `Generated { face, loop_index, edge_index }`: local trim/rebuild edge with no source topology; identity is intentionally fragment-scoped and must not be treated as shared topology.

### Regression expectations

- Keep characterization coverage for all three provenance categories whenever trim construction or rebuild logic changes.
- Preserve the repaired shell assembly, orientation, and fragment reuse regressions from `docs/plans/2026-03-28-truck-bop-trim-pipeline-repair-v2.md`.
- Keep `cyclic_edge_sequence_matches()` allocation-free on the reversed comparison path; do not reintroduce a temporary reversed vector for loop matching.

### Scope boundary

Future pave-filler work may build on this identity model, but it should not weaken the explicit provenance contract or collapse topology keys back into inferred `Option` combinations.

---

## 13. 方案 A 详细实施计划（Detailed Plan）

这一节把“方案 A：语义对齐优先，最小侵入接管”进一步细化到**可按会话执行、可按提交拆分、可逐批回归**的粒度。

### 13.1 总体执行原则

#### 原则 1：先固化数据层，再接算法层

不要先写 `section()/common()/cut()/fuse()` 外部 API；应先把以下内部事实层补齐：

- `FaceInfo(On / In / Sc)`
- `CommonBlock`
- `PaveBlock` 生命周期
- `EE / EF / FF` 的可落库 interference 语义
- `PaveFiller` 调度器

#### 原则 2：每一批只解决一类语义

不要把 `VE/EE/EF/FF` 一次打包实现。推荐每一批只解决一类“输入 -> DS -> 下游消费”的闭环。

#### 原则 3：每一批必须遵循红灯 -> 实现 -> 全量最小回归

每个批次的最小节奏都应是：

1. 先补 failing / characterization test；
2. 再写最小实现；
3. 再跑该批次的 focused tests；
4. 最后跑一轮 `trim.rs` 的保护性回归。

#### 原则 4：优先保持 `trim.rs` 作为后段事实消费者不破

短期目标不是重写 `trim.rs`，而是让它逐步消费更完整的 DS 事实。

---

### 13.2 建议拆分为 8 个提交批次

#### Batch 1：DS 骨架升级

**目标：** 先把占位型 `BopDs` 变成真实事实源。

**文件：**
- Modify: `truck-bop/src/bopds/face_info.rs`
- Modify: `truck-bop/src/bopds/common_block.rs`
- Modify: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/bopds/mod.rs`

**建议改动：**
- `FaceInfo` 从两字段升级为六字段：
  - `on_vertices`
  - `in_vertices`
  - `sc_vertices`
  - `on_pave_blocks`
  - `in_pave_blocks`
  - `sc_pave_blocks`
- 为 `FaceInfo` 增加最小 API：
  - `push_on_vertex(...)`
  - `push_in_vertex(...)`
  - `push_sc_vertex(...)`
  - `push_on_pave_block(...)`
  - `push_in_pave_block(...)`
  - `push_sc_pave_block(...)`
- `CommonBlock` 从单 `pave_block` 升级为：
  - `pave_blocks: Vec<PaveBlockId>`
  - `faces: Vec<FaceId>`
  - `representative_edge: Option<EdgeId>` 或等价 canonical 语义
- 在 `BopDs` 中增加：
  - `face_infos: FxHashMap<FaceId, FaceInfo>`
  - `common_blocks: Vec<CommonBlock>`
  - `pave_block_to_common_block: FxHashMap<PaveBlockId, CommonBlockId>`
  - `interference_pairs: FxHashMap<(ShapeId, ShapeId), usize>` 或等价去重表
- 保持旧接口尽量不删，只新增更强语义接口。

**测试：**
- `face_info_tracks_on_in_sc_separately`
- `face_info_deduplicates_repeated_block_registration`
- `common_block_can_group_multiple_pave_blocks`
- `bopds_can_bind_pave_block_to_common_block`
- `bopds_can_store_interference_pair_index`

**验收标准：**
- `BopDs` 能为任意 face 惰性创建 `FaceInfo`
- `CommonBlock` 不再局限单 block
- 不破坏现有 `BopDs` 测试

**建议提交消息：**
- `feat: extend bopds face and common block state`

---

#### Batch 2：`PaveBlock` 生命周期重构

**目标：** 把 `PaveBlock` 从静态段升级成“可追加 ext paves 并再次 split”的切分单元。

**文件：**
- Modify: `truck-bop/src/bopds/pave_block.rs`
- Modify: `truck-bop/src/bopds/mod.rs`

**建议改动：**
- 在 `PaveBlock` 中增加：
  - `ext_paves: Vec<Pave>`
  - `result_edge: Option<EdgeId>`
- `BopDs` 新增最小 API：
  - `append_ext_pave(block_id, pave)`
  - `split_pave_blocks_for_edge(edge_id)`
  - `rebuild_pave_blocks_from_paves(edge_id)`
- `append_ext_pave` 不直接生成新 edge，只负责把点挂到 block 上。
- `split_pave_blocks_for_edge` 负责：
  1. 取原始 block；
  2. 合并 start/end + ext paves；
  3. 排序去重；
  4. 生成新的子 blocks；
  5. 保留 `unsplittable` 语义。

**测试：**
- `split_pave_blocks_inserts_extra_paves_in_sorted_order`
- `split_pave_blocks_deduplicates_close_extra_paves`
- `split_pave_blocks_creates_n_minus_one_blocks_after_extra_paves`
- `unsplittable_micro_segment_is_preserved_but_not_split_again`

**验收标准：**
- 旧的 endpoint pave 逻辑继续成立
- `ext_paves` 能稳定进入排序链路
- `rebuild_pave_blocks_for_edge` 与 `split_pave_blocks_for_edge` 语义分工清楚

**建议提交消息：**
- `feat: add splittable pave block lifecycle`

---

#### Batch 3：VE 接入 block-split 闭环

**目标：** 让 `VE` 成为第一类真正走完整闭环的 interference。

**文件：**
- Modify: `truck-bop/src/intersect/ve.rs`
- Modify: `truck-bop/src/bopds/mod.rs`

**建议改动：**
- `intersect_ve(...)` 保留求交逻辑，但落库方式改为：
  1. push `VEInterference`
  2. 找到命中的 `PaveBlockId`
  3. 若命中 endpoint，则复用已有 pave
  4. 若命中内部参数，则 `append_ext_pave`
  5. 调用 `split_pave_blocks_for_edge(edge_id)`
- 不要再把 `VE` 当作只写 `paves` 的孤立事件。

**测试：**
- `ve_intersection_appends_extra_pave_into_matching_block`
- `ve_intersection_splits_edge_after_extra_pave_is_added`
- `ve_intersection_does_not_duplicate_existing_endpoint_pave`
- `ve_intersection_respects_parametric_tolerance_when_matching_block`

**验收标准：**
- `VE` 结果能稳定反映到 `pave_blocks_for_edge()`
- endpoint touch 不产生重复 split

**建议提交消息：**
- `feat: route ve intersections through pave block splitting`

---

#### Batch 4：EE 升级为 vertex-hit / overlap-hit

**目标：** 把 `EE` 从纯参数对返回值升级为能表达共享顶点与共享段的 DS 事实。

**文件：**
- Modify: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/intersect/ee.rs`
- Modify: `truck-bop/src/bopds/mod.rs`

**建议改动：**
- 将 `EEInterference` 改为枚举或等价结构：
  - `VertexHit { edge1, edge2, t_a, t_b, vertex }`
  - `OverlapHit { edge1, edge2, range_a, range_b }`
- 对 `VertexHit`：
  - 两边各自追加 ext pave
  - 两边都 split
- 对 `OverlapHit`：
  - 生成两侧成对的 `PaveBlock`
  - 把这对 blocks 归入 `CommonBlock`
- 让 `EE` 成为 `trim` 未来 shared boundary 的真正上游事实源。

**测试：**
- `ee_perpendicular_segments_create_shared_vertex_and_split_both_edges`
- `ee_endpoint_touch_reuses_existing_vertices_without_duplicate_split`
- `ee_colinear_overlap_creates_common_block_not_single_vertex_only`
- `ee_overlap_registers_faces_only_when_consumed_by_face_relations`

**验收标准：**
- 单点交和重合段两类语义分开表达
- 共线 overlap 不退化为多个离散顶点

**建议提交消息：**
- `feat: promote ee intersections to shared vertex and overlap facts`

---

#### Batch 5：EF 接入 `FaceInfo`

**目标：** 把 `EF` 从局部相交提升到面状态事实层。

**文件：**
- Modify: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/intersect/ef.rs`
- Modify: `truck-bop/src/bopds/mod.rs`

**建议改动：**
- 扩展 `EFInterference` 语义：
  - `PointHit`
  - `OverlapHit`
- `PointHit`：
  - 写 edge parameter pave
  - 更新 `FaceInfo.on_vertices` 或 `FaceInfo.in_vertices`
- `OverlapHit`：
  - 将 edge 对应 block 写入 `FaceInfo.on_pave_blocks`
  - 需要时建立 `CommonBlock`
- 先不追求所有复杂 surface case，优先让 plane + line / circle 的主路径跑通。

**测试：**
- `ef_point_intersection_updates_face_on_or_in_state`
- `ef_intersection_adds_edge_parameter_pave_for_split`
- `ef_edge_on_face_boundary_is_promoted_to_common_block`
- `ef_reuses_existing_pave_block_when_overlap_matches_boundary_range`

**验收标准：**
- `FaceInfo` 的 `On/In` 状态第一次被真实使用
- `EF` 不再只是孤立的参数事件

**建议提交消息：**
- `feat: connect ef intersections to face state and pave blocks`

---

#### Batch 6：FF 升级为 section-edge blocks

**目标：** 把 `FF` 的 `SectionCurve` 变成 trimming / section 可直接消费的 section-edge block 来源。

**文件：**
- Modify: `truck-bop/src/intersect/ff.rs`
- Modify: `truck-bop/src/bopds/interference.rs`
- Modify: `truck-bop/src/bopds/mod.rs`
- Modify: `truck-bop/src/trim.rs`

**建议改动：**
- 保留 `SectionCurve` 现有字段，但增加：
  - curve-local `paves`
  - curve-local block 构建辅助逻辑
- 把 `EF` 产生的 vertex、端点 vertex、closed-loop sampling vertex 投到 section curve 上
- 由 curve 上相邻 paves 生成 section-edge blocks
- 将这些 blocks 同时登记到两个 face 的 `FaceInfo.sc_pave_blocks`
- `build_trimming_loops()` 优先消费 `sc_pave_blocks`，必要时再回退到原 section polyline。

**测试：**
- `ff_section_curve_projects_ef_vertices_into_curve_paves`
- `ff_section_edge_is_registered_in_both_faces_sc_pool`
- `ff_open_section_preserves_distinct_start_end_vertices`
- `ff_closed_loop_section_enables_trimming_loop_construction`

**验收标准：**
- `FF` 结果第一次与 `trim.rs` 真正闭环
- 两个 owning faces 的 `sc` 事实保持一致

**建议提交消息：**
- `feat: convert ff section curves into section-edge block facts`

---

#### Batch 7：新增统一 `PaveFiller`

**目标：** 引入统一前置调度器，但先不改 public API 语义，只让内部主链跑通。

**文件：**
- Add: `truck-bop/src/pave_filler.rs`
- Modify: `truck-bop/src/lib.rs`
- Modify: `truck-bop/src/pipeline.rs`

**建议改动：**
- 新建 `PaveFiller` 或 `PaveFillerReport` 风格接口，最小包含：
  - `prepare_sources(...)`
  - `run_vv(...)`（可先空实现）
  - `run_ve(...)`
  - `run_ee(...)`
  - `run_vf(...)`
  - `run_ef(...)`
  - `run_ff(...)`
  - `split_pave_blocks(...)`
  - `make_split_edges(...)`
  - `build_trimming_loops(...)`
  - `build_split_faces(...)`
- `PipelineReport` 可升级为真正携带：
  - interferences count
  - split edge count
  - trimming loop count
  - split face count
- `PaveFiller` 先服务内部调用，不急于做复杂对外 API。

**测试：**
- `section_pipeline_runs_ve_ee_ef_ff_and_builds_split_edges`
- `pave_filler_report_counts_generated_section_curves_and_split_faces`
- `pave_filler_is_idempotent_for_repeated_runs_on_same_inputs`

**验收标准：**
- `VE/EE/EF/FF` 不再由外部调用方手工拼接
- `trim.rs` 能从统一前置流水线消费输入

**建议提交消息：**
- `feat: add unified pave filler orchestration pipeline`

---

#### Batch 8：按 `section -> common -> cut -> fuse` 接通 public API

**目标：** 在内部链路已经稳定后，再逐个接 public API。

**文件：**
- Modify: `truck-bop/src/lib.rs`
- Modify: `truck-bop/src/pipeline.rs`
- Modify: `truck-bop/src/trim.rs`

**建议改动：**
- `section()`：
  - 优先返回 FF/EF/EE 驱动的 section shell 或等价边集合装配结果
- `common()`：
  - 先复用 `build_split_faces + classify_split_faces_against_operand + select_split_faces_for_boolean_op`
- `cut()`：
  - 在 `common()` 稳定后接入
- `fuse()`：
  - 最后接入，重点关注 shared topology 复用与闭壳校验

**测试：**
- `section_two_boxes_returns_non_empty_shell`
- `common_two_overlapping_boxes_returns_closed_solid`
- `cut_box_minus_box_returns_closed_solid_with_expected_component_count`
- `fuse_two_touching_boxes_reuses_shared_topology_without_open_shell`

**验收标准：**
- 四个外部 API 不再返回 `NotImplemented`
- 至少 cuboid / plane-face 主路径能稳定通过

**建议提交消息：**
- `feat: wire public boolean apis through pave filler pipeline`

---

### 13.3 每批的最小验证命令

#### Batch 1-2

```bash
cargo test -p truck-bop bopds::tests -- --nocapture
cargo test -p truck-bop pave_block -- --nocapture
cargo check -p truck-bop
```

#### Batch 3-6

```bash
cargo test -p truck-bop intersect::ve -- --nocapture
cargo test -p truck-bop intersect::ee -- --nocapture
cargo test -p truck-bop intersect::ef -- --nocapture
cargo test -p truck-bop intersect::ff -- --nocapture
cargo test -p truck-bop trim::tests::boundary_loops_preserve_original_edge_provenance -- --exact
cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --exact
```

#### Batch 7-8

```bash
cargo test -p truck-bop trim::tests -- --nocapture
cargo test -p truck-bop pipeline::tests -- --nocapture
cargo test -p truck-bop
cargo check -p truck-bop
cargo check --release -p truck-bop
```

---

### 13.4 建议的会话安排

#### 会话 1：只做 Batch 1

输出目标：
- `FaceInfo / CommonBlock / BopDs` 结构升级
- DS 级测试绿灯

#### 会话 2：只做 Batch 2

输出目标：
- `PaveBlock` 生命周期跑通
- split block 测试绿灯

#### 会话 3：Batch 3 + Batch 4

输出目标：
- `VE / EE` 首次打通边级切分闭环

#### 会话 4：Batch 5 + Batch 6

输出目标：
- `EF / FF` 首次打通面级状态与 section-edge 闭环

#### 会话 5：Batch 7

输出目标：
- 统一 `PaveFiller` 主链成型

#### 会话 6：Batch 8 + 全量回归

输出目标：
- public API 首次可用
- 全量 `cargo test/check/check --release` 绿灯

---

### 13.5 必须保持的边界

#### 边界 1：不要破坏现有 provenance 合同

`TrimmingEdgeProvenance` / `TrimmingTopologyKey` 是当前 trim 主线已经收敛好的事实模型；新 pave-filler 工作应该建立在它之上，而不是绕过它另造一套隐式 identity。

#### 边界 2：不要让 `FF` 直接短路 `trim.rs`

即使 `FF` 先能生成 section polyline，也应尽快收敛到：

- `SectionCurve`
- `curve paves`
- `section-edge blocks`
- `FaceInfo.sc_pave_blocks`

而不是长期维持“FF 直接喂一根 polyline 给 trim”的双轨模型。

#### 边界 3：不要在 Batch 1-4 过早改 public API

否则会把“数据层稳定性问题”伪装成“布尔 API 语义问题”，增加排障成本。

---

### 13.6 建议的实施起点

如果下一轮就开始干实现，我建议**从 Batch 1 开始，不跳步**。

最小起手顺序：

1. 改 `FaceInfo`
2. 改 `CommonBlock`
3. 改 `BopDs` 持有结构
4. 先补 DS 级测试
5. 全绿后再进入 `PaveBlock`

原因很明确：

> 方案 A 的成功关键，不在于先做出一个能跑的 `section()`，而在于先把“交叉事实如何在仓库里表达”这件事定稳。

