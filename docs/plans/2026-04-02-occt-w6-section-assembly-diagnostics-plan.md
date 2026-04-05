# 2026-04-02 | W6-1 Section 组装失败诊断能力补充计划（truck-bop）

> 状态：进行中（W6-2）
> 上级文档：`docs/plans/2026-04-01-occt-bool-migration-master-plan-truck-new.md`

## 一、目标与范围

在保持 `section` 对外 API 行为不变（仍可返回空结果不阻断管线）的前提下，给 `section` 组装失败提供可复盘的内部诊断结果，方便后续定向修复 `assemble_shells` 的拓扑方向问题。

约束：
- 不改现网成功路径语义（`run_boolean_pipeline(BooleanOp::Section, ...)` 仍返回 `Ok(Vec::new())` 作为降级结果）；
- 不改变 `trim` 既有可验证行为；
- 所有新增诊断都走内部链路（仅用于测试/日志/后续修复）。

## 二、现状核对（基于当前代码）

- `truck-bop/src/trim.rs`
  - 新增内部诊断类型 `AssemblyFailure` 与 `AssemblyError`；
  - `assemble_shells` 通过 `assemble_shells_with_diagnostic` 封装；
  - `sew_shell_faces` 通过 `sew_shell_faces_with_diagnostic` 封装；
  - 当出现可定向边不足/开口壳/方向校验失败时，会返回可细分原因。
- `truck-bop/src/pipeline.rs`
  - 新增 `run_boolean_pipeline_with_diagnostics`；
  - `run_boolean_pipeline` 维持原签名调用降级接口；
  - section 失败时在内部返回 `Some(SectionAssemblyFailureSummary)`（不再只返回 `TopologyInvariantBroken`）。
- 回归测试
  - `pipeline::tests::run_boolean_pipeline_section_collects_diagnostic_summary_on_failure` 已可直接验证 summary 结构字段。

## 三、计划执行（Task List）

### W6-1.1（已完成）
- [x] 识别并抽取 `assemble_shells` 的失败路径；
- [x] 为失败路径定义最小诊断枚举：`NoOrientableCandidate / OpenShell / InvalidShellOrientation`；
- [x] 在 `trim.rs` 与 `pipeline.rs` 建立内部诊断通道。

### W6-1.2（已完成）
- [x] 消除 section 诊断输出中的临时性 `println!`（改为返回路径归档，不直接标准输出）；
- [x] 将 summary 字段接入明确回归场景（测试中已消费 `reason`、`selected_split_face_count`、`total_split_edges` 等字段）；
- [x] 在执行清单中补齐本次变更的回归验证记录。

### W6-2（已完成）
- [x] 增加 `section` 专用 `Failure ID`（用于日志与问题追踪）；
- [x] 为 `run_boolean_pipeline_with_diagnostics` 增加文档注释：失败时“降级返回 + summary 可选返回”的行为约定；
- [x] 补充 Section API 回退一致性测试：`section` 在无交点场景与装配失败场景都降级为 `UnsupportedGeometry`；
- [x] 把 `section` 成功路径恢复为更严格的前置约束：`section` 成功路径要求恰好返回 1 个 `Shell`，否则降级为 `UnsupportedGeometry`；
- [x] 新增单元测试覆盖“section 输入在结果返回多个壳时的降级边界”。

## 四、验收条件（当前里程碑）

1. 编译通过：
- `cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_section_collects_diagnostic_summary_on_failure -- --exact`
2. section 降级行为不变：
- `cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_section_returns_empty_for_now_for_overlapping_boxes -- --exact`
3. section 空结果非交点路径不应出现诊断摘要（新增约束）：
- `cargo test -p truck-bop pipeline::tests::run_boolean_pipeline_section_no_intersection_records_empty_without_diagnostic_summary -- --exact`
4. section 降级失败场景诊断可见：
- `SectionAssemblyFailureSummary` 中 `reason`、`selected_split_face_count`、`total_split_edges` 等至少有一类可复用字段被消费。

5. section 失败 ID 可复现（可用于日志聚合）：
- 复测同一失败复现场景两次，`failure_id` 一致。

6. section API 成功路径边界增强：
- `section` 在返回非空且非单壳结果时应显式降级为 `UnsupportedGeometry`（已按 `section` API 中 `if shells.len() != 1` 收口）。

7. section API 回退边界一致性：
- `cargo test -p truck-bop tests::section_api_returns_unsupported_for_non_intersecting_operands -- --exact`
- `cargo test -p truck-bop tests::section_api_returns_unsupported_when_assembly_fails -- --exact`
- `cargo test -p truck-bop tests::section_api_returns_unsupported_when_multiple_shells_exist -- --exact`

## 五、实施思路（Thought）

W6-1 的关键思路是“先保住现网行为，再加诊断”。  
当前阶段不要求立刻消灭 section 失败，而是把失败原因显式化，形成最小可用的修复闭环。  
下一步可把成功路径收紧并把 `failure_id` 与现有统一埋点/trace 方案衔接，形成可跨版本可复盘的 section 失败指标。

## 八、W6-2 收官记录（2026-04-02）

- 已补齐：`Failure ID`、`run_boolean_pipeline_with_diagnostics` 文档行为约定、section API fallback 三类边界测试（无交点/装配失败/多壳）。
- 当前验证结论：`section` API 在异常路径收敛为 `UnsupportedGeometry`，`common/cut/fuse` 不再返回 `NotImplemented`。
