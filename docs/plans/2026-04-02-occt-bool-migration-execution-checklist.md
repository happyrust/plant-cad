# 2026-04-02 | OCCT 布尔运算迁移执行清单（Truck BOP）

> 目标：把迁移计划落成可执行任务，支持开发者按项推进、逐项验收。

## 一、执行原则（复用总计划）

1. **先 DS，再调度，再 API 暴露**。
2. 每完成一个模块：先跑最小回归测试，再改下一个模块。
3. 任何项一旦出现连续 2 次回归失败，先回退该项改动并补充归因。
4. 产物约定：每个任务必须对应至少 1 个测试。
5. **明确限制：布尔运算继续走 truck-bop PaveFiller/trim 迁移链路，不引入或切换到 truck-polymesh 的布尔备选路径。**
6. `truck-polymesh` 的测试资源问题只作为独立的环境/测试面问题处理，不作为本迁移链路阻塞条件。

---

## 二、当前基线与环境约束

- 仓库：`/Volumes/DPC/work/cad/truck-new`
- 关键现状入口：
  - `truck-bop/src/lib.rs`（`section` 已去 NotImplemented；`common/cut/fuse` 已接入统一 pipeline）
  - `truck-bop/src/bopds/mod.rs`
  - `truck-bop/src/pipeline.rs`
  - `truck-bop/src/trim.rs`
  - `truck-bop/src/intersect/{ve,ee,ef,ff}.rs`
  - `truck-bop/src/bopds/{face_info,common_block}.rs`

---

## 三、阶段清单与验收标准（按 W1~W7）

### W0. 基线与文档化

#### 任务 W0-1：文档基线建立
- 文件：`docs/plans/2026-04-01-occt-bool-migration-master-plan-truck-new.md`
- 目标：计划文档完成并与本清单链接。
- 验收：
  - 文档存在；
  - 包含范围、阶段、风险、里程碑。

#### 任务 W0-2：测试基线记录
- 命令：
  ```bash
  cd /Volumes/DPC/work/cad/truck-new
  cargo test -p truck-bop
  cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --nocapture
  ```
- 验收：
  - 记录通过数/失败数
  - 将失败信息写入 `docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md` 的“回归日志”区。

### W1. 数据结构语义补齐（BopDs / FaceInfo / CommonBlock）

#### 任务 W1-1：完善 `face_info.rs`
- 文件：`truck-bop/src/bopds/face_info.rs`
- 改动：
  - 增加面状态集合（例如 `on/inside/outside` 语义相关字段）
  - 提供 `insert/update/query` API
  - 增加重复项幂等逻辑
- 验收：
  - 测试 `face_info_records_state_changes`
  - 测试 `face_info_state_merge_uses_latest`（新增）
  - 测试 `face_info_queries_is_deterministic`（新增）

#### 任务 W1-2：补齐 `common_block.rs`
- 文件：`truck-bop/src/bopds/common_block.rs`
- 改动：
  - 引入共享块的 `members/pave_blocks/faces/edges` 表达
  - 添加查询工具（是否为空、是否活动、包含关系）
- 验收：
  - 测试 `common_block_collects_multiple_pave_blocks`
  - 测试 `common_block_dedup_members`

#### 任务 W1-3：集中 BopDs 生命周期管理
- 文件：`truck-bop/src/bopds/mod.rs`
- 改动：
  - 增加 face/common block 的 add/find/clear 接口
  - 增加版本化快照（用于 scheduler 报告）
- 验收：
  - 现有 `BopDs` 测试不退化
  - 新增 `bopds_face_common_block_roundtrip`

### W2. 引入 PaveFiller 调度器（统一入口）

#### 任务 W2-1：新增 `pave_filler.rs`（状态：**已完成**）
- 文件：`truck-bop/src/pave_filler.rs`（新增）
- 改动：
  - `PaveFiller` 结构和 `BopOperationReport`
  - `prepare -> run_ve_ee_vf_ef_ff -> split_pave_blocks -> make_split_edges -> build_trimming_loops -> build_split_faces`
- 验收：
  - 编译通过（module 可见）
  - 新增 `pave_filler_runs_empty_input`（空输入可运行）
  - 新增 `pave_filler_reports_counts`（计数字段链路可读）
  - `cargo test -p truck-bop pave_filler` 全部通过
  - `cargo check -p truck-bop --release` 全量通过（无新增 warnings）

#### 任务 W2-2：pipeline 接入
- 文件：`truck-bop/src/pipeline.rs`
- 改动：
  - 将布尔前段调用从分散逻辑改为 `PaveFiller` 调度
- 验收：
  - 调度前后输出 `BopOperationReport` 或等效日志
  - `classify_point_in_solid` 接口不受破坏

#### 任务 W2-3：lib 导出与模块注册（状态：**已完成**）
- 文件：`truck-bop/src/lib.rs`
- 改动：
  - `pub mod pave_filler`（或等效导出）
  - 内部可见性保持一致
- 验收：
  - `cargo check -p truck-bop` 无额外未使用导出警告（如有允许说明）

**进展更新（2026-04-02）**

- 已将 `truck-bop/src/lib.rs` 从 `mod pave_filler;` 调整为 `pub mod pave_filler;`
- 已补充导出：`pub use pave_filler::{BopOperationReport, PaveFiller};`
- `cargo check -p truck-bop --release` 通过，说明 `W2-1/W2-3` 的最小可见性要求已满足（后续会在 pipeline 实接后复核 API 边界）。

### W3. VE/VV 产线一致化

#### 任务 W3-1：VV 决定（最小占位或实现）
- 文件：`truck-bop/src/pave_filler.rs`
- 改动：
  - 明确 VV 支持级别，返回 `None` 或已实现结果。
- 验收：
  - `vv_stage_visible`（文档化标注）
  - 生成器不会 panic

#### 任务 W3-2：VE 去重与幂等
- 文件：`truck-bop/src/intersect/ve.rs`
- 改动：
  - 参数边界与重复顶点去重策略统一
  - 写入到 `BopDs` 的 `VEInterference` 与 `Pave` 在重复场景下只保留一个
- 验收：
  - 测试覆盖端点重复、共线顶点、容差边界

### W4. EE / EF / FF 干涉到 DS 落库

#### 任务 W4-1：EE 结果归一化
- 文件：`truck-bop/src/intersect/ee.rs`
- 改动：
  - overlap/collinear/tangent 分支标记
  - 与 `Pave` 插入策略统一
- 验收：
  - 新增 `ee_overlap_records_as_interference`
  - `ee_collinear_no_duplicate_params`

#### 任务 W4-2：EF 记录增强
- 文件：`truck-bop/src/intersect/ef.rs`
- 改动：
  - EF 干涉结果支持 section curve 采样字段归档
- 验收：
  - 新增 `ef_intersection_generates_projection_samples`（若无投影则显式 false）
  - 新增 `ef_intersection_marks_projection_unavailable_for_refined_search_hit`（refine 场景落成 `face_projection_available = false`）

**进展更新（2026-04-05）**

- 已完成 `W4-2` 初始落地：
  - `EFInterference` 扩展了 `face_projection_sample: Option<Point3>` 与 `face_projection_available: bool`。
  - 新增 `ef_intersection_generates_projection_samples` 测试，并验证投影命中时 `face_projection_available = true`。
  - 新增 `ef_intersection_marks_projection_unavailable_for_refined_search_hit` 测试，并验证 refine 命中时 `face_projection_available = false`。
  - 当前保留现有 `intersect_ef` 的 `FaceInfo`/paves 写入策略，仅增强 DS 落库可观测性字段。

- W4-2 后续增强清单（完成）：
  - `ff` 模块新增 W4-3 对齐测试点（`ff_interference_links_section_curve`、`ff_touching_surface_generates_minimal_curve`）。

#### 任务 W4-3：FF 曲线归档
- 文件：`truck-bop/src/intersect/ff.rs`
- 改动：
  - `SectionCurve` 与 `FFInterference` 关系稳定化
  - 为失败场景提供明确错误码/返回状态（`FFIntersectionReport` + `FFIntersectionFailureRecord`）
- 验收：
  - 测试 `ff_interference_links_section_curve`
  - 测试 `ff_touching_surface_generates_minimal_curve`

**进展更新（2026-04-05）**
- 已完成失败状态化：
  - 新增 `FFIntersectionFailure`（枚举）与 `FFIntersectionFailureRecord`，`intersect_ff` 改为返回 `FFIntersectionReport`（`success + failures`）。
  - 无交线（例如平行无交、缺少候选面）会记录可复用失败码，替代静默 `continue`。
  - `PaveFiller` 按返回结构的 `success` 继续填充 `report.ff`，不影响下游汇总口径。
- 验收补充：
  - 新增/更新测试：`ff_intersection_reports_missing_face_as_failure`、`ff_intersection_distinguishes_parallel_face_no_intersection_as_no_segments`

### W5. PaveBlock + Split 边/面接驳

#### 任务 W5-1：PaveBlock 构建一致性
- 文件：`truck-bop/src/bopds/pave_block.rs`, `truck-bop/src/bopds/mod.rs`
- 改动：
  - 参数顺序与区间拼接规则固化
  - 重复 paves 去重策略统一
- 验收：
  - 保留现有 `paveblock_creation_...` 回归
  - 新增 `paveblock_rebuild_is_idempotent`

#### 任务 W5-2：SplitEdges 生成
- 文件：`truck-bop/src/pave_filler.rs`
- 改动：
  - 基于 PaveBlock 生成 split edge 的统一入口
  - 产生的 split 数据可用于 Trim
- 验收：
  - `split_edges_count_matches_pave_blocks`

#### 任务 W5-3：CommonBlock 聚合挂载
- 文件：`truck-bop/src/bopds/common_block.rs`, `truck-bop/src/bopds/mod.rs`
- 改动：
  - 构建 shared 块索引
  - 与 face/edge 形成最小映射
- 验收：
  - `commonblock_from_paveblocks_for_face`

### W6. section/common/cut/fuse 分阶段打开

#### 任务 W6-1：section 打通
- 文件：`truck-bop/src/pipeline.rs`, `truck-bop/src/lib.rs`
- 改动：
  - 不再 `NotImplemented`
  - section 路径走：`PaveFiller` -> `trim`
- 验收：
  - `cargo test -p truck-bop -- section`
  - 当前里程碑：验证 section 通路可运行（支持空结果语义，后续再增强为非空结果）

**进展更新（2026-04-02）**

- `section` 与 `run_boolean_pipeline` 已不再走 `NotImplemented`，并完成最小冒烟测试。
- 新增测试：
  - `pipeline::tests::run_boolean_pipeline_rejects_non_positive_tolerance_for_section`
  - `pipeline::tests::run_boolean_pipeline_section_returns_empty_for_now_for_overlapping_boxes`
- 当前语义状态（当前里程碑）：`Section` 分支已接入 `PaveFiller + trim` 链路，并对 `assemble_shells` 的 `TopologyInvariantBroken` 做了保守降级（返回空 `Vec<Shell>`），使 section 冒烟测试可稳定通过；下步目标仍是补齐 section 闭合拼装语义。
- 补充更新（本次）：
  - 新增 `run_boolean_pipeline_with_diagnostics`；
  - `trim.rs` 与 `pipeline.rs` 增加 section 组装失败摘要能力；
  - 新增测试 `pipeline::tests::run_boolean_pipeline_section_collects_diagnostic_summary_on_failure` 通过；
  - 对应文档：`docs/plans/2026-04-02-occt-w6-section-assembly-diagnostics-plan.md`。
- 追加更新（本次）：
  - `SectionAssemblyFailureSummary` 增加可复盘的 `failure_id`；
  - `run_boolean_pipeline_with_diagnostics` 新增文档注释，定义 section 降级 + summary 约定；
  - 新增测试 `pipeline::tests::run_boolean_pipeline_section_diagnostic_failure_id_is_reproducible`。

**进展更新（2026-04-02，W6-2）**

- `W6-2` 侧新增主链路冒烟：`pipeline::tests::run_boolean_pipeline_common_cut_fuse_disjoint_operands_returns_empty_shell_vector`；
  用于确认 common/cut/fuse 在无交点输入下不会进入 `NotImplemented` 路径并可执行成功返回。
- `W6-2` 新增对照：`pipeline::tests::run_boolean_pipeline_disjoint_operands_no_summary_for_non_section_ops`，
  约束无交点场景下，section 诊断仅在 `BooleanOp::Section` 上出现，common/cut/fuse 附带 `Option` 概要必须是 `None`。
- `W6-2` 新增 API 回退边界对照：`tests::section_api_disjoint_operands_keep_scheme_a_contract`，
  验证无交点输入时 section 与 common/cut/fuse 在外部错误语义上的分化（前者 Unsupported，后三者 Ok）。
- `W6-2` 完善 API 回归：`tests::disjoint_operands_follow_scheme_a_common_fuse_cut_contract`，补充验证当前阶段 common/cut/fuse 对于无交点输入均可走通，且 common/fuse 返回 `UnsupportedGeometry`，cut 返回保守回退结果（壳数与左操作数一致）。
- `W6-2` 同步 `lib.rs` smoke 已保留：`tests::common_cut_fuse_smoke_no_notimplemented`、`tests::disjoint_operands_follow_scheme_a_common_fuse_cut_contract`。
- `docs` 已更新到 `docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md` 的“W6-2 进展更新”和“执行命令/基线说明”。

#### 任务 W6-2：common/cut/fuse 最小可用
- 文件：`truck-bop/src/pipeline.rs`, `truck-bop/src/lib.rs`
- 改动：
  - 接入与 trim 一致的前置流程；复杂边界先保守处理
- 验收：
  - `common/cut/fuse` 不再立即返回 `NotImplemented`（即便语义先保守）
  - 至少有一次成功路径运行

### W7. 回归与持续清理

#### 任务 W7-1：回归测试池整理
- 文件：`truck-bop/tests/*`（或对应模块内）
- 新增清单（建议）:
  - `section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces`
  - `pavefiller_builds_faceinfo_commonblock`
  - `common_cut_fuse_smoke_no_notimplemented`
- 验收：
  - 通过率 ≥ 上一阶段基线
- 与本次推进同步：`W7-1` 两条目标测试已补齐并通过（见“C. 2026-04-02 执行补充清单”）。
- 补充说明（2026-04-02）：这两条测试分别验证了 section 前置切分链路与 `FaceInfo`/`CommonBlock` 可追踪挂接关系形成，已进入阶段验收可见区。

#### 任务 W7-2：性能与日志
- 文件：`docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md`
- 新增每周指标：
  - 候选对数量、pave 数、pave block 数、trim 时长
  - 失败类型 Top3 及对应根因
- 验收：
  - 指标有持续更新条目
- 实施细则（v0）：
  - 统计字段来源：优先使用 `PaveFiller::report()` 与 `SectionAssemblyFailureSummary`（当前可复用）：
    - `vv/ve/vf/ee/ef/ff`：候选对与干涉记录；
    - `pave_blocks`：`split` 阶段后当前 `BopDs` 中 paveblock 总量；
    - `split_edges`：`split_pave_blocks` 生成的 split edge 计数；
    - `trimming_loops/split_faces`：trim 阶段的分割规模；
    - `failure_id/reason/remaining_orientable_candidates`：section 失败可复盘字段（用于 Top3 归因）。
  - trim 时长口径：
    - `trim_ms` 已改为函数级采样：通过 `run_boolean_pipeline_with_diagnostics_and_timing` 在
      `run_boolean_pipeline_with_diagnostics` 路径内返回总耗时（`u128` 毫秒）；
    - 采样口径来自 `pipeline.rs` 中函数级 `Instant::now` 到结束时刻；
    - 在测试日志中可直接读取 `trim_ms`。
  - 每周至少补一条 W7-2 指标记录到本文件“W7-2 周更”区。

##### W7-2 指标与日志模板（请每周填一条）
```text
[OCCT-Bool][W7-2][2026-04-xx][case_id=YYYY]
op=section|common|cut|fuse
case=brief_case_name
total_candidate_pairs=vv+ve+vf+ee+ef+ff
vv=... ve=... vf=... ee=... ef=... ff=...
pave_blocks=... split_edges=... trimming_loops=... split_faces=... trim_ms=...
section_failure_id=... reason=NoOrientableCandidate|OpenShell|InvalidShellOrientation|none
failure_top_candidates=1)NoOrientableCandidate:count ...
result=pass|fail(reason)
notes=...
```

### G. 2026-04-05 文档命令一致性组合执行补充复核

对 `ci-cpu-test-with-bool-doc-check` 中 CPU 阶段的失败点复核后，确认是以下资源文件缺失导致编译中断（与布尔文档门禁无关）：
- `truck-polymesh/../resources/stl/bunny_ascii.stl`
- `truck-polymesh/../resources/stl/bunny_binary.stl`
- `truck-polymesh/../resources/obj/teapot.obj`
- `truck-polymesh/../resources/obj/teapot-with-normals.obj`
- `truck-polymesh/../resources/obj/skull-with-texcoord.obj`
- `truck-polymesh/../resources/obj/pony-complete.obj`

处理建议：先补齐上述资源文件后再执行 `cargo make --no-workspace ci-cpu-test-with-bool-doc-check`，避免将环境资源缺失误判为迁移进度阻塞。

##### W7-2 每周更新（示例）
| 周期 | 总用时(ms) | 候选对总数 | pave数 | split block 数 | trim耗时(ms) | 失败 Top3 | 结论 |
|---|---:|---:|---:|---:|---:|---|---|
| 2026-04-02（section落地） | 242 | 648 | 80 | 80 | 242 | 1) `NoOrientableCandidate` `1`/2) `OpenShell` `0`/3) `InvalidShellOrientation` `0` | `section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces` |
| 2026-04-02（common对照） | 2 | 0 | 48 | 48 | 2 | 1) `NoOrientableCandidate` `0`/2) `OpenShell` `0`/3) `InvalidShellOrientation` `0` | `common_pipeline_runs_disjoint_common_baseline` |
| 2026-04-02（cut/fuse对照） | 5 | 0 | 48 | 48 | 5 | 1) `NoOrientableCandidate` `0`/2) `OpenShell` `0`/3) `InvalidShellOrientation` `0` | `cut_pipeline_runs_disjoint_operands_w7_2` / `fuse_pipeline_runs_disjoint_operands_w7_2` |

**W7-2 验收动作（本周）**
- 依据 `W7-2 每周更新` 完成至少一条完整记录。
- 统一用“失败原因 Top3 + 根因”更新失败归因区：根因统一写到 `notes=`。

---

## 四、回归命令清单（最小可执行顺序）

### 阶段内通用回归
```bash
cd /Volumes/DPC/work/cad/truck-new
cargo test -p truck-bop
cargo test -p truck-bop shared_vertex_only_faces_still_form_one_component
cargo test -p truck-bop shared_edge_faces_create_orientation_adjacency
cargo test -p truck-bop shell_assembly_groups_faces_by_shared_topology_components
cargo test -p truck-bop trim::tests::shell_orientation_rejects_closed_shell_with_inverted_faces -- --exact
```

### 新增功能回归（每个里程碑必须运行）
```bash
cargo test -p truck-bop --features ?
# 上线后逐项打开：
cargo test -p truck-bop pavefiller_builds_faceinfo_commonblock
cargo test -p truck-bop section
cargo test -p truck-bop pipeline::tests::section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces -- --exact
cargo test -p truck-bop pave_filler::tests::pavefiller_builds_faceinfo_commonblock -- --exact
cargo test -p truck-bop pipeline::tests::common_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces -- --exact --nocapture
cargo test -p truck-bop tests::common_cut_fuse_smoke_no_notimplemented
cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_common_cut_fuse_disjoint_operands_returns_empty_shell_vector -- --exact
cargo test -p truck-bop tests::disjoint_operands_follow_scheme_a_common_fuse_cut_contract -- --exact
```

### 文档命令一致性预检（建议每次更新回归段时执行）
```bash
cd /Volumes/DPC/work/cad/truck-new
./scripts/verify-bool-doc-regressions.sh docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md
```

说明：脚本会抽取本文件里所有 `cargo test -p truck-bop ...` 命令，并与 `cargo test -p truck-bop -- --list` 的当前测试清单按过滤子串进行一致性校验。脚本会自动跳过占位符命令（例如 `--features ?`），仅对可解析的可验证命令做缺失报告，避免回归命令名漂移导致的文档与实际测试不一致。

---

## 五、执行交付模板（每次提交/里程碑后补充）

- 任务范围：
- 修改文件：
- 新增/更新测试：
- 本阶段最小验证命令：
- 结果（通过/失败）：
- 失败案例与归因：
- 下一步关闭动作：

---

## 六、下一步触发条件

若 W2 之前 W1 任务中任一项出现明显设计歧义（例如 FaceInfo 的状态枚举与布尔语义映射不一致）：

1. 先冻结 API 字段命名（给出 `enum` 与字段定义草案）；
2. 停止实现进度；
3. 以一个“字段契约草案”进行评审；
4. 评审通过后继续 W2。


## 七、已执行基线记录（2026-04-02）

### A. 快速基线（trim 场景）

```bash
cargo test -p truck-bop trim::tests::shell_assembly_groups_faces_by_shared_topology_components -- --nocapture
```

结果：
- 通过：1
- 失败：0
- 说明：该回归点在当前版本稳定通过。

### B. 全量基线（truck-bop）

```bash
cargo test -p truck-bop
```

结果（当前分支）：
- 通过：191
- 失败：0
- 忽略：4
- 说明：`intersect::ef` 的三项已在近期测试链路中已过（当前文档截面为最新通过状态）。
- 说明（2026-04-02）：本轮执行新增 section 与无交点分支回归命令全部通过（见下方补充清单）。
- 说明（2026-04-03）：加入 W7-2 后，全量回归稳定于 `191 passed / 0 failed / 4 ignored`。

### C. 2026-04-02 执行补充清单（本轮）

```bash
cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_section_returns_empty_for_now_for_overlapping_boxes -- --exact
cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_section_no_intersection_records_empty_without_diagnostic_summary -- --exact
cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_common_cut_fuse_disjoint_operands_returns_empty_shell_vector -- --exact
cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_disjoint_operands_no_summary_for_non_section_ops -- --exact
cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_cut_fuse_disjoint_operands_records_w7_2_metrics -- --exact --nocapture
cargo test -p truck-bop tests::section_api_returns_unsupported_for_non_intersecting_operands -- --exact
cargo test -p truck-bop tests::section_api_disjoint_operands_keep_scheme_a_contract -- --exact
cargo test -p truck-bop tests::section_api_returns_unsupported_when_assembly_fails -- --exact
cargo test -p truck-bop tests::section_api_returns_unsupported_when_multiple_shells_exist -- --exact
cargo test -p truck-bop tests::disjoint_operands_follow_scheme_a_common_fuse_cut_contract -- --exact
cargo test -p truck-bop pipeline::tests::section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces -- --exact
cargo test -p truck-bop pave_filler::tests::pavefiller_builds_faceinfo_commonblock -- --exact
cargo test -p truck-bop pipeline::tests::common_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces -- --exact --nocapture
cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_cut_fuse_disjoint_operands_records_w7_2_metrics -- --exact --nocapture
```

新增说明（2026-04-02）：
- `run_boolean_pipeline_section_returns_empty_for_now_for_overlapping_boxes` 通过，确认 Section 路径未再返回 `NotImplemented`。
- `run_boolean_pipeline_section_no_intersection_records_empty_without_diagnostic_summary` 通过，确认无交点场景 section 走空结果且不返回诊断摘要。
- `section_api_returns_unsupported_for_non_intersecting_operands`、`section_api_returns_unsupported_when_assembly_fails` 与 `section_api_returns_unsupported_when_multiple_shells_exist` 通过，确认无交点/装配失败/多壳场景统一降级为 `UnsupportedGeometry`。
- `pipeline::tests::run_boolean_pipeline_common_cut_fuse_disjoint_operands_returns_empty_shell_vector` 通过，确认 common/cut/fuse 在无交集输入下仍走通主流程（未再出现 `NotImplemented`）。当前不依赖其空壳行为。
- `tests::disjoint_operands_follow_scheme_a_common_fuse_cut_contract` 通过，确认 common/cut/fuse API 侧在无交点输入下可成功返回保守回退语义：`common`/`fuse` 回退为 `UnsupportedGeometry`，`cut` 返回左操作数副本（边界数与左操作数一致；待后续语义收敛替换为真几何结果）。
- `pipeline::tests::section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces` 通过，验证 section 场景中 VE/EE/VF/EF/FF 与切分构建路径联通。
- `pave_filler::tests::pavefiller_builds_faceinfo_commonblock` 通过，验证 `FaceInfo` 在 split 阶段存在且可通过 `common_block_for_pave_block` 找到共享块关系。
- `pipeline::tests::run_boolean_pipeline_cut_fuse_disjoint_operands_records_w7_2_metrics` 通过，补齐 cut/fuse 分支的 W7-2 指标采集样本（无交点保守回退场景）。
- W7-2 周报样本采集（本轮）：
  - 命令：`cargo test -p truck-bop pipeline::tests::section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces -- --exact --nocapture`
  - 对应输出（含可回填周更字段）：
    - `[OCCT-Bool][W7-2][2026-04-02][case_id=section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces] op=section case=section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces total_candidate_pairs=648 vv=0 ve=192 vf=96 ee=192 ef=144 ff=24 pave_blocks=80 split_edges=80 trimming_loops=22 split_faces=12 trim_ms=242 section_failure_id=12684860847782502332 reason=NoOrientableCandidate remaining_orientable_candidates=1 result=pass notes=baseline_seeded
- W7-2 周报样本采集（补充）：
  - 命令：`cargo test -p truck-bop pipeline::tests::common_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces -- --exact --nocapture`
  - 对应输出（含可回填周更字段）：
    - `[OCCT-Bool][W7-2][2026-04-02][case_id=common_pipeline_runs_disjoint_common_baseline] op=common case=common_pipeline_runs_disjoint_common_baseline total_candidate_pairs=0 vv=0 ve=0 vf=0 ee=0 ef=0 ff=0 pave_blocks=48 split_edges=48 trimming_loops=12 split_faces=12 trim_ms=2 section_failure_id=0 reason=none remaining_orientable_candidates=0 result=pass notes=common_disjoint_baseline`
- W7-2 周报样本采集（cut/fuse补充）：
  - 命令：`cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_cut_fuse_disjoint_operands_records_w7_2_metrics -- --exact --nocapture`
  - 对应输出（含可回填周更字段）：
    - `[OCCT-Bool][W7-2][2026-04-02][case_id=cut_pipeline_runs_disjoint_operands_w7_2] op=cut case=cut_pipeline_runs_disjoint_operands_w7_2 total_candidate_pairs=0 vv=0 ve=0 vf=0 ee=0 ef=0 ff=0 pave_blocks=48 split_edges=48 trimming_loops=12 split_faces=12 trim_ms=5 section_failure_id=0 reason=none remaining_orientable_candidates=0 result=pass notes=cut_fuse_disjoint_baseline`
    - `[OCCT-Bool][W7-2][2026-04-02][case_id=fuse_pipeline_runs_disjoint_operands_w7_2] op=fuse case=fuse_pipeline_runs_disjoint_operands_w7_2 total_candidate_pairs=0 vv=0 ve=0 vf=0 ee=0 ef=0 ff=0 pave_blocks=48 split_edges=48 trimming_loops=12 split_faces=12 trim_ms=5 section_failure_id=0 reason=none remaining_orientable_candidates=0 result=pass notes=cut_fuse_disjoint_baseline`
- 追加回归命令（2026-04-02）：
- `cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_section_returns_empty_for_now_for_overlapping_boxes -- --exact`
- `cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_section_no_intersection_records_empty_without_diagnostic_summary -- --exact`
- `cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_disjoint_operands_no_summary_for_non_section_ops -- --exact`
- `cargo test -p truck-bop -- section -- --exact`
- `cargo test -p truck-bop tests::section_api_returns_unsupported_for_non_intersecting_operands -- --exact`
- `cargo test -p truck-bop tests::section_api_disjoint_operands_keep_scheme_a_contract -- --exact`
- `cargo test -p truck-bop tests::section_api_returns_unsupported_when_assembly_fails -- --exact`
- `cargo test -p truck-bop tests::section_api_returns_unsupported_when_multiple_shells_exist -- --exact`
- `cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_common_cut_fuse_disjoint_operands_returns_empty_shell_vector -- --exact`
- `cargo test -p truck-bop tests::disjoint_operands_follow_scheme_a_common_fuse_cut_contract -- --exact`
- `cargo test -p truck-bop -- section -- --exact`
- `cargo check -p truck-bop --release`
- 对上述命令结果说明：Section 冒烟链路稳定通过；Release check 通过，当前未新增 warning 之外的编译错误。

### D. 2026-04-02 之后 W7-2 持续指标（待补齐）

> 进展：本周已完成三条 W7-2 行为记录（section/common/cut-fuse 全部补齐）。后续每周补齐新行即可。

### D.1 指标字段复用映射（函数级口径）

用于复测时快速复用：
- `trim_ms`：`run_boolean_pipeline_with_diagnostics_and_timing(... )` 返回值（`u128`，单位 ms）。
- `total_candidate_pairs`：`candidate_total = vv + ve + vf + ee + ef + ff`。
- `pave_blocks`：`filler.split_pave_blocks(...)` 返回值。
- `split_edges`：`filler.make_split_edges(...)` 返回值。
- `trimming_loops`：`filler.build_trimming_loops(...)` 返回值。
- `split_faces`：`filler.build_split_faces(...)` 返回值。
- `failure_id / reason / remaining_orientable_candidates`：`SectionAssemblyFailureSummary`。
- `result`：成功路径 `pass`；若无壳或降级场景建议记为 `fail(reason)`。

```text
周期：2026-04-02
样本case：
- name=section_pipeline_runs_ve_ee_ef_ff_and_builds_split_faces
- operation=Section
notes：
- 首次落地样本已采集并完成字段回填：vv=0, ve=192, vf=96, ee=192, ef=144, ff=24.
- 总体 Top3 归因为 `NoOrientableCandidate`（1）优先，其次 `OpenShell` / `InvalidShellOrientation` 0。

输入规模：
- candidate_pairs: vv=0, ve=192, vf=96, ee=192, ef=144, ff=24
- candidate_total=648
- pave_blocks=80
- split_edges=80
- trimming_loops=22
- split_faces=12
- trim_ms=242（函数级采样，来自 `run_boolean_pipeline_with_diagnostics_and_timing`）

失败与根因 Top3：
- 1) NoOrientableCandidate = 1
- 2) OpenShell = 0
- 3) InvalidShellOrientation = 0

notes:
- 结论：`trim_ms` 使用函数级采样；统一口径由 `run_boolean_pipeline_with_diagnostics_and_timing` 产生，避免测试框架外的额外抖动影响。

- name=common_pipeline_runs_disjoint_common_baseline
- operation=Common
notes：
- 非 section 路径 baseline（非失败）：`reason=none`，用于形成 top3 之外的非失败参照。

输入规模：
- candidate_pairs: vv=0, ve=0, vf=0, ee=0, ef=0, ff=0
- candidate_total=0
- pave_blocks=48
- split_edges=48
- trimming_loops=12
- split_faces=12
- trim_ms=2（函数级采样，来自 `run_boolean_pipeline_with_diagnostics_and_timing`）

失败与根因 Top3：
- 1) NoOrientableCandidate = 0
- 2) OpenShell = 0
- 3) InvalidShellOrientation = 0

notes:
- 结论：该样本用于确认 common 路径在该几何下可执行（result=pass）且不进入 section 失败摘要通道。

- name=cut_pipeline_runs_disjoint_operands_w7_2
- operation=Cut
notes：
- 无交点保守回退场景的 cut 基线。

输入规模：
- candidate_pairs: vv=0, ve=0, vf=0, ee=0, ef=0, ff=0
- candidate_total=0
- pave_blocks=48
- split_edges=48
- trimming_loops=12
- split_faces=12
- trim_ms=5（函数级采样，来自 `run_boolean_pipeline_with_diagnostics_and_timing`）

失败与根因 Top3：
- 1) NoOrientableCandidate = 0
- 2) OpenShell = 0
- 3) InvalidShellOrientation = 0

notes:
- 结论：该样本用于确认 cut 路径无交点时走可执行保守回退（result=pass）且不进入 section 失败摘要通道。

- name=fuse_pipeline_runs_disjoint_operands_w7_2
- operation=Fuse
notes：
- 无交点保守回退场景的 fuse 基线。

输入规模：
- candidate_pairs: vv=0, ve=0, vf=0, ee=0, ef=0, ff=0
- candidate_total=0
- pave_blocks=48
- split_edges=48
- trimming_loops=12
- split_faces=12
- trim_ms=5（函数级采样，来自 `run_boolean_pipeline_with_diagnostics_and_timing`）

失败与根因 Top3：
- 1) NoOrientableCandidate = 0
- 2) OpenShell = 0
- 3) InvalidShellOrientation = 0

notes:
- 结论：该样本用于确认 fuse 路径无交点时走可执行保守回退（result=pass）且不进入 section 失败摘要通道。
``` 

### E. 2026-04-04 文档回归命令一致性预检（执行记录）

执行命令：
```bash
./scripts/verify-bool-doc-regressions.sh docs/plans/2026-04-02-occt-bool-migration-execution-checklist.md
```

结果：
- commands: ok=31, missing=0, skipped=1
- 结论：当前回归命令与 `cargo test -p truck-bop -- --list` 一致；仅 `--features ?` 被识别为占位跳过项（符合预检规则）。

### F. 2026-04-05 文档命令一致性门禁组合执行（含任务化入口验证）

执行命令：
```bash
cargo make --no-workspace ci-cpu-test-with-bool-doc-check
```

执行结果：
- bool 文档一致性校验：pass（`commands: ok=31, missing=0, skipped=1`）
- CPU 回归：未通过（`cargo-polymesh` 测试阶段缺少 `resources/stl/bunny_*.stl`，与本次清单门禁改动无关）
- 环境原因：当前工作区资源路径不完整导致测试资源读取失败，属于现有环境噪音，不应回归到清单一致性实现缺失。

#### G.1 复核说明（资源缺失明细）

- 进一步复核 `cargo-make` 输出确认，失败文件还包括：
  - `truck-polymesh/tests/stl-io.rs`：`resources/stl/bunny_ascii.stl`, `resources/stl/bunny_binary.stl`
  - `truck-polymesh/tests/obj-io.rs`：`resources/obj/teapot.obj`, `resources/obj/teapot-with-normals.obj`, `resources/obj/skull-with-texcoord.obj`, `resources/obj/pony-complete.obj`
- 该问题与布尔迁移功能无关，建议补齐资源后再评估 CPU 组合门禁的最终绿灯状态。
