# 2026-04-01 | OCCT 布尔运算核心数据结构与算法迁移总体计划（truck-bop → truck-new）

> 状态：草案（可执行）

## 实施计划（Implementation Plan）

### 1. 目标与范围

本计划旨在将 **OCCT `BOPAlgo_PaveFiller` + `BOPDS_DS`** 的核心职责迁移到 `truck-new` 的 `truck-bop`，实现以下目标：

1. 用统一“切分-切边-切面”流水线替代当前分散的交互结果拼装。
2. 在 `truck-bop` 内形成可复用的布尔事实数据层（`Pave / PaveBlock / CommonBlock / FaceInfo / Interference`）。
3. 打通 `section / common / cut / fuse` 的主链路（至少不再是 `NotImplemented`）。
4. 维持现有 `trim.rs` 后链路能力，尽量最小侵入。

范围约束：
- 不重写几何内核算子本身；优先复用现有 `intersect::*` 与现有拓扑类型。
- 首先实现“正确性优先”的语义对齐，后续再做局部性能治理。
- 与 `truck-bop` 现有 `classify_point_in_solid`、`broad_phase` 保持兼容，不在本计划第一阶段做大规模 API 兼容重构。

### 2. 已验证现状（事实锚点）

#### 2.1 truck-bop 现有基础（可复用）

- `truck-bop/src/bopds/pave.rs`：已具备 `Pave{edge, vertex, parameter, tolerance}`。
- `truck-bop/src/bopds/pave_block.rs`：已有 `PaveBlock` 基础与重建逻辑。
- `truck-bop/src/bopds/mod.rs`：已有 `BopDs`，持有 `interferences / paves / pave_blocks`，但缺少完整的 `FaceInfo / CommonBlock` 语义。
- `truck-bop/src/bopds/interference.rs`：已定义 VE/EE/EF/FF/SectionCurve/TrimmingLoop 等干涉结构并支持表级持有。
- `truck-bop/src/intersect/ve.rs` 与 `ee.rs/ef.rs/ff.rs`：已有交集计算函数，并已在部分场景下向 `BopDs` 写入 `pave`/`interference`。
- `truck-bop/src/trim.rs`：已有 `build_trimming_loops -> build_split_faces -> sew -> assemble` 链路。
- `truck-bop/src/lib.rs`：`common/fuse/cut/section` 当前尚未打通（`NotImplemented`）。
- `truck-bop/src/pipeline.rs`：点分类为 `AABB` 启发式，现状与完整实体分类算法尚有差距。

#### 2.2 OCCT 侧可借鉴骨架（用于映射）

- `BOPDS_DS`：事实层（shape、interference、pave/block/common/face state）
- `BOPAlgo_PaveFiller`：统一调度器（`Prepare -> VV -> VE -> EE -> VF -> EF -> FF -> MakeSplitEdges -> MakeBlocks -> MakePCurves -> ProcessDE`）
- `BOPDS_Pave` / `BOPDS_PaveBlock`：边上参数节点与参数段
- `BOPDS_CommonBlock`：面公共边界与共享族管理
- `BOPDS_FaceInfo`：面分类/状态（Inside/On/Out）+ 约束与候选集

### 3. 总体技术策略

1. **先数据层后算法层后接口层（DS-first）**
   - 先把 `BopDs` 完整化：FaceInfo/CommonBlock/section结果写入一致。
2. **先语义对齐后性能优化**
   - 初始目标让几何结果稳定，再逐步优化 broad phase、排序、重建算法。
3. **单职责模块化**
   - 新增 `pave_filler` 只做调度与产物拼接；
   - `intersect/*` 只负责几何相交本体；
   - `trim.rs` 保持后处理责任。

---

## 任务清单（Task List）

### W0. 基线与接口冻结（1 周内）

1. **新增迁移追踪文档与验收模板**
   - 产物：`docs/plans/2026-04-01-occt-bool-migration-master-plan-truck-new.md`（当前文件）
   - 产物：`docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md`
   - 内容：阶段目标、里程碑、回归清单、性能监控点。

2. **建立可复现验证基线**
   - 运行：
     ```bash
     cargo test -p truck-bop
     cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --nocapture
     ```
   - 记录基线时间、通过率、失败用例与失败特征。

3. **建立文档回归命令一致性门禁（建议每次更新回归段前执行）**
   - 运行：
     ```bash
     ./scripts/verify-bool-doc-regressions.sh docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md
     ```
   - 机制说明：
     - 抽取执行清单里 `cargo test -p truck-bop ...` 的命令；
     - 与 `cargo test -p truck-bop -- --list` 当前测试名做过滤子串对齐；
     - 自动跳过占位符命令（如 `--features ?`）。
   - 验收：
     - 输出 `missing=0`；
     - 未通过不得将清单作为“当前可执行”提交到里程碑。
   - CI/脚本化入口：
     - `cargo make --no-workspace bool-doc-regressions-check`
     - `cargo make --no-workspace ci-cpu-test-with-bool-doc-check`（包含预检 + CPU 回归，便于里程碑前做一次性校验）

### W1. BopDs 语义加固（2 周内）

1. **补齐 `FaceInfo` 状态模型**
   - 文件：`truck-bop/src/bopds/face_info.rs`
   - 目标：
     - 增加与 OCCT 对齐的状态集合（如 `inside` / `outside` / `on` 分类池）
     - 建立对面片参与布尔的状态更新 API（append/update/query）
   - 测试：`face_info_records_state_changes`、`face_info_state_merges_duplicates`。

2. **补齐 `CommonBlock` 共享分组模型**
   - 文件：`truck-bop/src/bopds/common_block.rs`
   - 目标：
     - 支持“多个 `PaveBlock` 组成公共块”
     - 支持 `extent / is_active / witness_edges / witness_faces` 等检索字段。
   - 测试：`common_block_collects_multiple_pave_blocks`

3. **扩展 `BopDs` 访问器（保持 API 稳定）**
   - 文件：`truck-bop/src/bopds/mod.rs`
   - 目标：统一 `get/set` `paves / pave_blocks / face_infos / common_blocks / interferences / section_curves`。

### W2. 引入 PaveFiller 调度器（2 周内）

1. **新增统一调度模块**
   - 文件：`truck-bop/src/pave_filler.rs`（新建）
   - API（建议）：
     - `prepare(&mut self, operands: &[Operand])`
     - `run_ve_ee_vf_ef_ff(&mut self, broad_pairs: CandidatePairs)`
     - `fill_paves(&mut self)`
     - `split_pave_blocks(&mut self)`
     - `make_split_edges(&mut self)`
     - `build_trimming_loops(&mut self)`
     - `build_split_faces(&mut self)`

2. **把 `intersect::*` 的调用统一到 scheduler**
   - 文件：`truck-bop/src/pave_filler.rs`
   - 文件：`truck-bop/src/pipeline.rs`（按布尔 op 调用 `PaveFiller`）

3. **暴露调度结果快照接口**
   - 文件：`truck-bop/src/pave_filler.rs`
   - 返回值：`BopOperationReport`（含 counts of VE/EE/EF/FF/pave/block/split counts）

### W3. VV/VE 产线（1 周）

1. **VV 作为占位接口明确化（或最小实现）**
   - 文件：`truck-bop/src/pave_filler.rs`
   - 若暂不完整实现：明确 `Option` 与记录日志，避免 silent fail。

2. **VE：确认与 `BopDs` 挂钩一致性**
   - 文件：`truck-bop/src/intersect/ve.rs`
   - 任务：
     - 确保 VE 的 `parameter` 计算统一 tolerances 与重复顶点去重策略
     - 产生的 `Pave` 与 `VEInterference` 都能回填到 `BopDs`。
   - 测试：
     - `ve_intersection_detects_vertex_on_edge_and_generates_pave`（复用增强）
     - 新增共线/端点重复场景。

### W4. EE/EF/FF 与 SectionCurve 落库（2 周）

1. **EE：双参数区间段落到 PaveBlock 的入口语义**
   - 文件：`truck-bop/src/intersect/ee.rs`
   - 目标：
     - 确保 overlap/collinear/tangent 分支可区分并形成可观测统计。
     - 不同边对的 `EEInterference` 与 `Pave` 集合写入策略一致。
   - 输出：`ee_interference_to_paveblocks_report`。

2. **EF / FF：把 section 信息转为可复用实体**
   - 文件：`truck-bop/src/intersect/ef.rs`, `truck-bop/src/intersect/ff.rs`
   - 目标：
     - `FFInterference` 关联 `SectionCurveId`
     - `SectionCurve` 记录 face 参数域采样与投影可用性
     - `trim.rs` 能消费到 `TrimmingLoop` 输入所需数据。

3. **新增统一“结果一致性”断言集**
   - 文件：`truck-bop/tests/*` 或模块内测试
   - 目标：不同接口路径产生同一输入下等价干涉数量。

### W5. PaveBlock / SplitEdge / CommonBlock 串接（2 周）

1. **重建 `rebuild_paves_for_edges` 的行为一致性**
   - 文件：`truck-bop/src/bopds/mod.rs`, `truck-bop/src/bopds/pave_block.rs`
   - 目标：参数顺序去重、重复 pave 合并、反向参数归一化。

2. **生成并验证 SplitEdges**
   - 文件：`truck-bop/src/pave_filler.rs`（新增）
   - 目标：基于 `PaveBlock` 生成分段边，不破坏已有 `split` 语义。

3. **在 `CommonBlock` 中挂载面边共享关系**
   - 文件：`truck-bop/src/bopds/common_block.rs`, `truck-bop/src/bopds/mod.rs`
   - 目标：构建 `shared blocks` 索引，服务后续 `FaceInfo` 与面片拼接。

### W6. Trim 接驳与 section/common/cut/fuse 分阶段打开（2 周）

1. **section 路径迁移**
   - 文件：`truck-bop/src/pipeline.rs`, `truck-bop/src/lib.rs`
   - 目标：由 PaveFiller 报告驱动 `build_trimming_loops -> build_split_faces`。
   - 验收：`section` 返回可见结构，不再 `NotImplemented`。

2. **common/cut/fuse 的最小可用接入**
   - 文件：`truck-bop/src/pipeline.rs`
   - 目标：
     - 首期只做与 trim 后链路兼容的最小闭环；
     - 复杂拓扑行为按 feature flag 或阶段任务继续补齐。

3. **`FaceInfo` 状态和布尔结果一致性**
   - 文件：`truck-bop/src/bopds/face_info.rs`
   - 目标：按 `BooleanOp` 方向生成结果面片候选，保证切分结果可回溯。

### W7. 回归与性能收敛（持续）

1. **测试矩阵迁移**
   - `truck-bop` 当前已有测试保留：
     - `shared_vertex_only_faces_still_form_one_component`
     - `shared_edge_faces_create_orientation_adjacency`
     - `shell_assembly_groups_faces_by_shared_topology_components`
     - `trim::tests::shell_assembly_groups_faces_by_shared_topology_components`
   - 新增迁移测试：
     - `section_pipeline_runs_ve_ee_ef_ff_and_builds_trim`
     - `pavefiller_builds_faceinfo_commonblock`
     - `common_cut_fuse_smoke_no_notimplemented`

2. **性能观察**
   - `broad_phase.rs` 现有 O(n²) 形状对枚举在中小模型可接受，下一阶段接入更强候选剔除。
   - 指标：候选对数量、Pave 数、PaveBlock 数、Trim 运行时。

3. **文档同步**
   - 同步更新 `docs/plans/*` 与 `README`（如有对外说明）。

### W8. 风险清单与回退策略（并行）

1. **风险：与现有 `intersect` 逻辑存在重复写入**
   - 对策：将 `BopDs` 写入集中到 `PaveFiller` 阶段，建立幂等检查。

2. **风险：现有 Trim 链路假设不完全成立**
   - 对策：保持 `trim` 入口兼容层（adapter）并持续回归。

3. **风险：section/common/cut/fuse 需分期接入导致功能短板**
   - 对策：阶段性交付 `section` 最低可用，再逐步扩展 common/cut/fuse。

4. **回退策略**
   - 任何阶段出现连续 2 次回归红，回退到上一个稳定 commit；保留 `pave_filler` API 但降级到当前兼容路径，避免影响现网。

---

## 里程碑与验证标准

### 里程碑 A（W2）
- 完成 `BopDs` 与 `FaceInfo/CommonBlock` 结构化升级。
- 验收：新增结构在单元测试中可构建、可查询。

### 里程碑 B（W4）
- `PaveFiller` 完成 VE/EE/EF/FF 调度骨架并可产出 `interference` 结果。
- 验收：新增报告字段均有数值，含空输入/无交集场景。

### 里程碑 C（W6）
- `section` 不再 `NotImplemented`，可走 end-to-end `trim`。
- 验收：关键回归通过；至少 2 类复杂壳体拼接场景稳定。

### 里程碑 D（W7）
- `common/cut/fuse` 打通最小路径，形成可观察结果。
- 验收：CLI/API 示例可拿到非空输出（即使尚未覆盖全部边界语义）。

---

## 依赖与顺序（Implementation Path）

```text
BopDs 数据结构增强
  -> PaveFiller 调度器
    -> VE/EE/EF/FF 干涉写入一致
      -> PaveBlock 与 SplitEdge
        -> CommonBlock/FaceInfo
          -> Trim 接驳
            -> section/common/cut/fuse 打开
```

---

## 具体执行任务（Task List）

- [ ] 1. 更新 `truck-bop/src/bopds/face_info.rs`：`FaceInfo` 增加状态表与查询 API。
- [ ] 2. 更新 `truck-bop/src/bopds/common_block.rs`：补齐共享块语义与读写能力。
- [ ] 3. 更新 `truck-bop/src/bopds/mod.rs`：新增 face/common 块生命周期方法。
- [ ] 4. 新增 `truck-bop/src/pave_filler.rs`：实现 `PaveFiller` 调度与 `OperationReport`。
- [ ] 5. 修改 `truck-bop/src/intersect/ee.rs`：补齐 overlap/同构边界写入策略。
- [ ] 6. 修改 `truck-bop/src/intersect/ef.rs`：完善 EF 与面分类/边界采样落库。
- [ ] 7. 修改 `truck-bop/src/intersect/ff.rs`：完善 section curve 与 FF 干涉归档。
- [ ] 8. 增加 `truck-bop/src/pipeline.rs` 中对 `PaveFiller` 的调用。
- [ ] 9. 在 `truck-bop/src/lib.rs` 暴露 `common / cut / section / fuse` 至新主链路。
- [ ] 10. 补充/重构测试集，覆盖：交点产出、pave 构建、trim 驱动与 API 非 NotImplemented。

---

## Thought（思路与决策依据）

我选择“**先 DS 对齐，再调度接管，再 API 开放**”的顺序，核心考虑是：

1. 当前 `truck-bop` 已经有足够几何交点能力，但缺的是“统一事实源 + 分发者”；先补这个是成本最低且复用性最高。
2. OCCT 的 `PaveFiller` 本质不是“单一函数”，而是一组稳定的状态机化阶段；直接硬抄流程很容易产生与现有模块不协调问题。
3. `trim.rs` 已经经过多次 trim 相关修复，重建可复用的后段流水线会更安全。
4. 以 W 级阶段分拆可在每次迭代输出可用增量：先 section 可视化可行，再扩展 common/cut/fuse。
5. 计划按“能运行、可回退、易验证”原则执行，避免一次性大改导致调试成本过高。


- 生成时间：2026-04-01
- 维护原则：每次完成一个 W 阶段，先 `cargo test` 再更新本计划与变更记录。

---

## 附录：配套执行文档（新增）

为配合本总计划，已新增以下可执行子文档：

- `docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md`
  - 阶段化任务清单、验收标准、回归命令、基线日志。
- `docs/plans/2026-04-02-occt-w1-faceinfo-commonblock-contract.md`
  - W1（FaceInfo / CommonBlock）接口草案与迁移前契约。
- `docs/plans/2026-04-02-occt-w6-section-assembly-diagnostics-plan.md`
  - W6-1 section 组装失败诊断能力与下一步收敛动作。

在 W1 开始实施前，以上三份文档作为“提请审核”产物（当前 W6 相关请先看对应章节文档）。
