# truck-bop 布尔运算移植 — 下一步开发计划

> **日期：** 2026-04-13
> **基于：** `MISSION-truck-bop-port.md` 全量审核 + `cargo test -p truck-bop` 实测结果
> **当前基线：** 169 passed, 1 failed, 2 ignored; 19 编译警告

---

## 当前状态摘要

### 已完成的里程碑

- **M0 (Design Ready)**: ✅ 架构文档、ADR、能力矩阵均已完成
- **M1 (DS & Splitting Ready)**: ✅ `truck-bop` 可构建，typed IDs / Arena / BopDs 全部就位
- **M2 (MVP Boolean Pipeline)**: ✅ `common` 和 `fuse` 可端到端运行

### 部分达成的里程碑

- **M3 (Full Operation Set)**: ⚠️ `cut` 已实现，`section` 仍返回 `NotImplemented`
- **M4 (Validation Ready)**: ⚠️ 回归套件在建，有 1 个测试失败

### 未开始的里程碑

- **M5 (Robustness Upgrade)**: ❌

### ISSUES.md 中记录的 Bug 状态

| Issue | 严重度 | 状态 | 说明 |
|-------|--------|------|------|
| Issue 1 — `uv_to_model_point` 丢失 Z 坐标 | P0 | **已修复** | `merge_equivalent_vertices` 已改为通过 surface map 做真实曲面求值 |
| Issue 2 — `SPHint::from_face` 使用 XY 代替 UV | P0 | **已修复** | `vf.rs` 中已改用 `surface.search_nearest_parameter()` |
| Issue 3 — `classify_point_in_solid` 仅支持盒体 | P0 | **已修复** | `pipeline.rs` 已实现三级分类：AABB → ray-casting → nearest-face |
| Issue 4 — `vertex_ids_for_polyline` ID 碰撞 | P1 | **部分缓解** | 已改为递增 counter，但 counter 未与 `BopDs::next_generated_vertex_id` 统一 |

### 当前测试失败

```
FAILED: trim::tests::boolean_selection_fuse_selects_outside_and_boundary_fragments_from_both_operands
    assertion `left == right` failed: left: 2, right: 3
```

**根因已定位：** `trim.rs:1516` 中 fuse 选择逻辑为 `matches!(classification, PointClassification::Outside)`，
只选 `Outside` 而完全不选 `OnBoundary`。测试期望 fuse 选中两方 `Outside` + 一方 `OnBoundary`
（与 OCCT 语义一致：共面片只从一方计入，避免重复）。

### 任务依赖关系

```
A1 (fuse 修复) ─── 无依赖，单行修复
      │
      ├──→ A3 (ISSUES.md 更新) ─── 依赖 A1 完成确认基线
      ├──→ A4 (警告清理) ─── 独立
      │
      ├──→ B1 (section 实现) ─── 依赖 A1 稳定基线
      ├──→ B2 (曲面体测试) ─── 依赖 A1 + B1
      └──→ B3 (shapeops 对比) ─── 依赖 A1

A2 (ID 统一) ─── 独立于 A1，但需要穿透 trim.rs 调用链
      │
      └──→ C2 (PaveBlock 边分裂) ─── 依赖 A2 的 ID 一致性
```

---

## 阶段 A — 短期修复（稳定当前基线）

**目标：** `cargo test -p truck-bop` 全绿，`cargo check --release -p truck-bop` 零警告

**预估工期：** 2~3 天

**优先级排列原则：** A1 为最高优先级（阻塞所有下游任务），A2 可与 A1 并行但涉及面更广，A3/A4 为收尾工作。

### A1. 修复 fuse 选择逻辑测试失败 ⟵ **最高优先级**

- **文件：** `truck-bop/src/trim.rs`
- **精确位置：** `should_select_split_face` 函数，第 1516 行
- **根因：** 当前实现为：
  ```rust
  BooleanOp::Fuse => matches!(classification, PointClassification::Outside),
  ```
  只选 `Outside`，遗漏了 `OnBoundary`。
- **OCCT 语义参考：** fuse 应保留双方的 `Outside` fragment + **一方**的 `OnBoundary` fragment（通常取 operand 0 的 OnBoundary，避免共面片重复计入）。
- **修复方案：** 将 fuse 分支改为与 cut 类似的分 operand 策略：
  ```rust
  BooleanOp::Fuse => {
      if split_face.operand_rank == 0 {
          matches!(
              classification,
              PointClassification::Outside | PointClassification::OnBoundary
          )
      } else {
          matches!(classification, PointClassification::Outside)
      }
  }
  ```
- **影响范围：** 仅此一处，无副作用
- **验证命令：**
  ```bash
  cargo test -p truck-bop boolean_selection_fuse -- --exact
  cargo test -p truck-bop -- --lib  # 全量回归
  ```
- **退出条件：** `boolean_selection_fuse_selects_outside_and_boundary_fragments_from_both_operands` 通过

### A2. 统一 vertex ID 分配到 BopDs

- **问题本质：** `vertex_ids_for_polyline` (trim.rs:1556) 使用调用者传入的 `&mut u32` counter，而非 `BopDs::next_generated_vertex_id`。虽已不再使用算术公式，但 counter 初始值由 `build_trimming_loops` 内部硬编码，与 BopDs 的 ID 空间存在潜在冲突。
- **调用链分析：**
  ```
  build_trimming_loops(&mut BopDs, ...)
    └─→ build_loops_for_face(face_id, face, section_curves, tol, registry, &mut vertex_counter)
          ├─→ boundary_loops(face, section_curves, tol, registry, &mut vertex_counter)
          │     └─→ vertex_ids_for_polyline(&mut counter, polyline)
          └─→ section_loops(section_curves, face_id, tol, &mut vertex_counter)
                └─→ vertex_ids_for_polyline(&mut counter, polyline)
  ```
- **修复方案：**
  1. 将 `vertex_counter` 替换为 `&mut BopDs`（或从中借出 `&mut u32` 指向 `next_generated_vertex_id`）
  2. `vertex_ids_for_polyline` 签名改为 `fn vertex_ids_for_polyline(ds: &mut BopDs, ...) -> Vec<VertexId>`，内部调用 `ds.next_generated_vertex_id` 递增
  3. 删除 `build_trimming_loops` 中的局部 counter 变量
- **涉及文件：**
  - `truck-bop/src/trim.rs` — `build_trimming_loops`, `build_loops_for_face`, `boundary_loops`, `section_loops`, `vertex_ids_for_polyline`
- **验证命令：**
  ```bash
  cargo test -p truck-bop -- --lib
  ```
- **退出条件：** 所有 vertex ID 来自 BopDs 统一分配，trim.rs 内无独立 counter

### A3. 更新 ISSUES.md 状态

- 将 Issue 1/2/3 标记为 **resolved**：
  - Issue 1：已通过 `merge_equivalent_vertices` 重构修复（surfaces map 做真实曲面求值）
  - Issue 2：`vf.rs` 中 `SPHint::from_face` 已改用 `surface.search_nearest_parameter()`
  - Issue 3：`pipeline.rs` 已实现 AABB → ray-casting → nearest-face 三级分类
- Issue 4 在 A2 完成后标记为 **resolved**
- **依赖：** A1 完成后执行（确认基线稳定）

### A4. 清理编译警告

- `trim.rs:1773` — `suspicious_double_ref_op`：`edge.clone()` 作用于 `&&Edge`，应改为 `(*edge).clone()` 或解引用
- 补全公开 API 缺失的 `missing_docs`（主要集中在 `bopds/pave_block.rs` 的 `split_edge` / `set_split_edge` 方法）
- 确认 `PipelineReport` / `Arena` / `CommonBlock` / `FaceInfo` 的 `#[allow(dead_code)]` 标注仍合理
- **可与 A1/A2 并行执行**

### 阶段 A 退出标准

- `cargo test -p truck-bop` — 0 failed
- `cargo check --release -p truck-bop` — 0 warnings（dependency future-incompat 除外）

---

## 阶段 B — 中期补全（达成 M3 里程碑）

**目标：** `section` 操作可用，曲面体端到端测试通过，`truck-shapeops` 对比回归矩阵建立

**预估工期：** 1~2 周

### B1. 实现 `section` 操作

- **涉及文件：**
  - `truck-bop/src/lib.rs` — 替换 `section()` 的 `NotImplemented` 存根
  - `truck-bop/src/trim.rs` — 可能需要新增 `assemble_section_shell()` 独立于 `assemble_shells()`
  - `truck-bop/src/bopds/interference.rs` — `SectionCurve` 已有定义，需确认字段完整性
- **实现思路：**
  1. 复用 `run_boolean_pipeline` 的前半段（注册 shapes → 广相 → 六级干涉 → F/F section curve 提取）
  2. 从 `BopDs` 中提取所有 `SectionCurve` 记录
  3. 将 section curves 转换为 truck `Edge` 对象
  4. 组装为 `Wire` → `Face`（可能是退化 face）→ `Shell`
  5. **关键区别：** section 结果是**开壳**，不能经过 `assemble_shells()` 的 closed-shell 强校验
- **技术风险：**
  - section curves 可能是多段不连续曲线，需要处理分段
  - 需要明确 section Shell 中 face 的曲面定义（可能是 section curve 所在的交线面片）
- **依赖：** A1 完成（确保基线稳定）
- **验证命令：**
  ```bash
  cargo test -p truck-bop section -- --exact
  ```
- **退出条件：** `section(box, cylinder, tol)` 返回有效 Shell，`section(box, box, tol)` 返回共面面片

### B2. 补充曲面体端到端测试

按 `MISSION-truck-bop-port.md` 第 14 节要求覆盖：

- box vs cylinder — common / fuse / cut
- sphere vs sphere — common / fuse
- box cut by cylinder
- coplanar touching boxes
- tangent face contact
- near-coincident edges / micro segments

**文件：** `truck-bop/src/lib.rs` tests 模块或新增 `truck-bop/tests/integration.rs`

### B3. 建立 `truck-shapeops` 对比回归

- 对代表性场景同时调用 `truck-shapeops::and/or` 和 `truck-bop::common/fuse`
- 比较输出拓扑计数（faces、edges、vertices 数量）
- 可作为 integration test 或独立 benchmark
- **文件：** `truck-bop/tests/cross_check.rs` 或类似位置

### 阶段 B 退出标准

- `section` 操作不再返回 `NotImplemented`
- 至少 6 个曲面体端到端测试通过
- `truck-shapeops` vs `truck-bop` 对比回归矩阵存在且绿色

---

## 阶段 C — 长期强化（M4 ~ M5）

**目标：** 回归套件稳定，鲁棒性和性能达到生产可用水平

**预估工期：** 持续迭代

### C1. 显式公差增长追踪

- 在 `BopDs` 中记录每次 vertex/edge/face tolerance 的增长
- split 和 classification 逻辑消费记录的公差而非全局值
- 对应 `MISSION-truck-bop-port.md` 第 9 节要求

### C2. PaveBlock 驱动的边分裂流程

- 当前管线跳过了经典 Pave → PaveBlock → edge split 路径
- 补全 `truck-bop/src/split/edge_splitter.rs`（计划第 7 节 crate layout 中已规划）
- 使边分裂可回溯、可幂等

### C3. 改进 F/F 相交策略

- 当前 `intersect/ff.rs` 使用临时方案（mesh/polyline fallback）
- 目标：引入解析或混合 NURBS-NURBS 面面求交
- 对应计划中"delivery order #10: improved analytic or hybrid F/F implementation"

### C4. 性能分析与优化

- profiling 热路径：pair generation → F/F intersection → trim → rebuild
- 优化 `merge_equivalent_vertices` 的 O(n²) 邻接构建（可改为空间哈希或 KD-tree）
- 对应 Phase 9 中的性能基线目标

### C5. Provenance 增强

- 扩展 `ProvenanceMap` 覆盖范围至 edge-level
- 为 CAD 用户界面提供"哪个输入面/边产生了这个输出面/边"的查询能力

### 阶段 C 退出标准

- M4（回归套件稳定 + 对比完成）达成
- M5（改进 F/F + 难 case 稳定化 + 性能基线）达成
- `MISSION-truck-bop-port.md` 第 13 节验收标准全部满足

---

## 时间线建议

```
       Day 1          Day 2-3         Week 1-2          Week 3+
   ┌─────────┐    ┌──────────┐    ┌──────────────┐    ┌──────────┐
   │ A1 fuse │──→ │ A2 ID    │──→ │ B1 section   │──→ │ C1~C5    │
   │ (0.5天) │    │ (1天)    │    │ B2 曲面体测试│    │ 持续迭代 │
   │         │    │ A3 ISSUES│    │ B3 对比回归  │    │          │
   │         │    │ A4 警告  │    │              │    │          │
   └─────────┘    └──────────┘    └──────────────┘    └──────────┘
      阶段 A           阶段 A          阶段 B            阶段 C
```

## 风险提醒

1. **F/F 交集仍是最大技术风险** — 当前临时方案对曲面体的覆盖有限，阶段 B 测试可能暴露新问题
2. **section 操作的拓扑复杂度** — 交线面片是开壳，不能走 `assemble_shells()` 的 closed-shell 校验路径，需要独立组装逻辑
3. **`truck-shapeops` 对比可能揭示语义差异** — `and`/`or` 与 `common`/`fuse` 在边界 `OnBoundary` 处理上的微妙差别（A1 修复正是此类问题）
4. **公差传播** — 当前全局公差模型在复杂几何下可能导致级联失败（对应 MISSION 第 15 节 Risk 2）
5. **vertex ID 空间冲突** — A2 未完成前，大型模型（>100 面）可能触发 ID 碰撞导致难以调试的拓扑损坏
