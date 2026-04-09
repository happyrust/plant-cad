# CAD 布尔运算底层架构对比教程

> **你将学到：**
> - OCCT 和 Truck 两个库的布尔运算底层架构全貌
> - 核心数据结构（Pave/PaveBlock/Interference）的设计思想与对应关系
> - 两套流水线（Pipeline）的执行流程差异
> - 从 C++ 到 Rust 的迁移中遇到的工程取舍
>
> **前置知识：** B-Rep（边界表示）基础概念、基本的 Rust/C++ 读码能力
>
> **阅读时间：** 25–35 分钟 ｜ **难度：** 中级–高级

---

## 1. 项目概览

### 1.1 OCCT — OpenCASCADE Technology

OCCT 是工业级 B-Rep 内核，C++ 实现，历经 **30+ 年**迭代。布尔运算核心位于 **TKBO** 模块：

```
src/ModelingAlgorithms/TKBO/
├── BOPAlgo/      ← 算法层（PaveFiller、Builder、BOP）
│   ├── BOPAlgo_PaveFiller.cxx          # 交集计算核心（分 11 个文件！）
│   ├── BOPAlgo_Builder.cxx             # General Fuse 基类
│   ├── BOPAlgo_BOP.cxx                 # 布尔运算主入口
│   ├── BOPAlgo_BuilderFace.cxx         # 面重建
│   ├── BOPAlgo_BuilderSolid.cxx        # 体重建
│   └── ...（~40 个 .cxx/.hxx）
├── BOPDS/        ← 数据结构层
│   ├── BOPDS_DS.cxx                    # 核心数据集
│   ├── BOPDS_PaveBlock.cxx             # Pave 块
│   ├── BOPDS_Pave.cxx                  # Pave 点
│   ├── BOPDS_Interf.hxx                # 干涉信息
│   ├── BOPDS_Iterator.cxx              # 形状对迭代器
│   └── ...（~20 个文件）
├── BOPTools/     ← 工具层（BoxTree、AlgoTools）
├── IntTools/     ← 底层交集计算（曲线×曲线、曲线×曲面、曲面×曲面）
└── BRepAlgoAPI/  ← 高层 API（Common, Cut, Fuse, Section, Splitter）
```

代码规模：**约 30 万行 C++**，97 个头文件。

### 1.2 Truck-new — Rust CAD 内核

Truck 是 Rust 实现的现代 B-Rep 内核。布尔运算有**两套实现**并存：

**旧实现 — `truck-shapeops/transversal`（多边形三角化方法）：**

```
truck-shapeops/src/transversal/
├── intersection_curve/    ← 交集曲线计算
├── divide_face/           ← 面分割
├── faces_classification/  ← 面分类（射线法）
├── loops_store/           ← 循环存储
├── polyline_construction/ ← 多边线构建
└── integrate/             ← 集成布尔运算（对外暴露 and/or）
```

**新实现 — `truck-bop`（移植 OCCT 架构）：**

```
truck-bop/src/
├── bopds/                 ← 数据结构（移植自 BOPDS）
│   ├── pave.rs            # Pave 点
│   ├── pave_block.rs      # Pave 块
│   ├── interference.rs    # 干涉表
│   ├── common_block.rs    # 公共块
│   ├── face_info.rs       # 面信息
│   ├── ids.rs             # 类型安全的 ID 系统
│   └── shape_info.rs      # 形状注册信息
├── intersect/             ← 交集算法（对应 IntTools）
│   ├── vv.rs, ve.rs       # Vertex×Vertex, Vertex×Edge
│   ├── ee.rs, ef.rs       # Edge×Edge, Edge×Face
│   ├── vf.rs, ff.rs       # Vertex×Face, Face×Face
├── pipeline.rs            ← 流水线（点分类、射线投射）
├── trim.rs                ← 裁剪与拓扑重建（169KB，最复杂的文件）
├── broad_phase.rs         ← 宽相位碰撞检测
├── bounding.rs            ← 包围盒计算
├── options.rs             ← 运行时选项
└── lib.rs                 ← 对外 API（common/fuse/cut/section stubs）
```

---

## 2. 核心数据结构对比

布尔运算的基石是**中间数据结构**——它存储交集计算的全部结果，供后续分割和构建阶段消费。

### 2.1 Pave — 边上的参数化分段点

**概念：** Pave（铺设点）表示一条边（Edge）上的一个关键位置——可以是原始端点，也可以是与另一条边/面相交产生的新点。

**OCCT (`BOPDS_Pave`)：**

```cpp
// BOPDS_Pave.hxx（简化）
class BOPDS_Pave {
  Standard_Integer myIndex;     // 顶点在 DS 中的全局索引
  Standard_Real    myParameter; // 边曲线上的参数值 t
};
```

**Truck-bop (`Pave`)：**

```rust
// bopds/pave.rs
pub struct Pave {
    pub edge: EdgeId,        // 所属边的 ID
    pub vertex: VertexId,    // 对应顶点 ID
    pub parameter: f64,      // 边曲线上的参数值
    pub tolerance: f64,      // 容差（每个 Pave 独立携带）
}
```

**关键差异：**

| 维度 | OCCT | Truck-bop |
|------|------|-----------|
| ID 系统 | 全局整数索引 | newtype 包装的类型安全 ID（`EdgeId(u32)`, `VertexId(u32)`） |
| 容差归属 | 存储在 DS 级别 | **每个 Pave 独立携带**，支持局部精度调整 |
| 验证 | 运行时检查 | 构造函数 `Pave::new()` 返回 `Result<Self, BopError>`，编译期保证合法性 |

> **Checkpoint 1：** Pave 是整个布尔运算数据流的原子单位。一条边上的所有 Pave 按参数排序后，相邻 Pave 之间就构成了一个 PaveBlock。

### 2.2 PaveBlock — 边的参数化分段

**概念：** PaveBlock 是一条边被 Pave 点切割后的一个**连续子段**。如果一条边上有 N 个 Pave 点，就会产生 N-1 个 PaveBlock。

**OCCT (`BOPDS_PaveBlock`)：**

```cpp
class BOPDS_PaveBlock : public Standard_Transient {
  BOPDS_Pave myPave1, myPave2;              // 起止 Pave
  Standard_Integer myEdge;                    // 分割后的新边索引
  Standard_Integer myOriginalEdge;            // 原始边索引
  BOPDS_ListOfPaveBlock myExtPaves;          // 中间 Pave（用于子分割）
  Handle(BOPDS_CommonBlock) myCommonBlock;   // 公共块引用
};
```

**Truck-bop (`PaveBlock`)：**

```rust
// bopds/pave_block.rs
pub struct PaveBlock {
    pub original_edge: EdgeId,
    pub start_vertex: VertexId,
    pub end_vertex: VertexId,
    pub param_range: (f64, f64),
    pub unsplittable: bool,         // 微小段标记
    ext_paves: Vec<Pave>,           // 中间 Pave
    split_edge: Option<EdgeId>,     // 分割后的新边
}
```

**亮点对比：**

- **`unsplittable` 标记（Truck 独有）：** 当 `param_range` 跨度小于 `parametric_tol` 时自动标记，防止微小段导致数值崩溃——这是 OCCT 中需要用户手动处理的退化情况，Truck 直接在数据结构层面解决。

- **`update()` 方法：** Truck 的 PaveBlock 可以通过 `ext_paves` 进一步细分，返回子 PaveBlock 列表：

```rust
// 一个 [0.0, 1.0] 的 PaveBlock 在 t=0.3 和 t=0.7 处加入 ext_pave 后：
let result = block.update(1.0e-8, PaveBlockId(0));
// → [(0, PB[0.0→0.3]), (1, PB[0.3→0.7]), (2, PB[0.7→1.0])]
```

### 2.3 Interference — 干涉信息

**概念：** 两个拓扑元素（Vertex、Edge、Face）相交产生的几何信息记录。

**OCCT (`BOPDS_Interf`)：** 使用继承体系：

```
BOPDS_Interf (基类)
├── BOPDS_InterfVV   (点×点)
├── BOPDS_InterfVE   (点×边)
├── BOPDS_InterfVF   (点×面)
├── BOPDS_InterfEE   (边×边)
├── BOPDS_InterfEF   (边×面)
├── BOPDS_InterfFF   (面×面)
└── BOPDS_InterfVZ/EZ/FZ  (带容差区域)
```

**Truck-bop：** 使用扁平结构 + InterferenceTable：

```rust
// interference.rs（核心类型）
pub struct VVInterference { pub vertex1: VertexId, pub vertex2: VertexId }
pub struct VEInterference { pub vertex: VertexId, pub edge: EdgeId, pub parameter: f64 }
pub struct EEInterference { pub edge1: EdgeId, pub edge2: EdgeId, pub t_a: f64, pub t_b: f64 }
pub struct EFInterference { pub edge: EdgeId, pub face: FaceId, pub parameter: f64, pub surface_parameters: (f64, f64) }
pub struct FFInterference { pub face1: FaceId, pub face2: FaceId, pub section_curve: SectionCurveId }

// 统一存储
pub struct InterferenceTable {
    pub vv: Vec<VVInterference>,
    pub ve: Vec<VEInterference>,
    // ... 共 11 个 Vec
    pub section_curves: Vec<SectionCurve>,
    pub trimming_loops: Vec<TrimmingLoop>,
    pub split_faces: Vec<SplitFace>,
    pub merged_vertices: Vec<MergedVertex>,
    pub sewn_paths: Vec<SewnPath>,
}
```

**设计哲学差异：**

| OCCT | Truck-bop |
|------|-----------|
| 继承层次（C++ `Standard_Transient`） | 扁平枚举 + 独立 struct |
| 通过全局索引关联 | 通过 newtype ID 关联（`EdgeId`, `FaceId` 等编译期类型检查） |
| 干涉与附加产物（section curve、trimming loop 等）**分散**存储 | **全部集中**在 `InterferenceTable` 中 |

### 2.4 BopDs / BOPDS_DS — 核心数据集

**这是两个库布尔运算的"心脏"**——汇聚所有注册的形状、计算出的干涉、Pave/PaveBlock，以及后续分割需要的中间结果。

**OCCT (`BOPDS_DS`)：** ~3000 行 C++，包含形状表、干涉池、Pave 管理、BoundingBox 缓存等。

**Truck-bop (`BopDs`)：** ~500 行 Rust：

```rust
pub struct BopDs {
    options: BopOptions,
    shapes: Vec<ShapeInfo>,
    vertex_to_shape: FxHashMap<VertexId, ShapeId>,
    edge_to_shape: FxHashMap<EdgeId, ShapeId>,
    face_to_shape: FxHashMap<FaceId, ShapeId>,
    interferences: InterferenceTable,
    paves: Vec<Pave>,
    pave_blocks: Vec<PaveBlock>,
    common_blocks: Vec<CommonBlock>,
    pave_block_to_common_block: FxHashMap<PaveBlockId, CommonBlockId>,
    face_info_pool: FxHashMap<FaceId, FaceInfo>,
    // ID 分配器
    next_section_curve_id: u32,
    next_common_block_id: u32,
    pub(crate) next_generated_vertex_id: u32,
    pub(crate) boundary_edge_registry: SourceBoundaryEdgeRegistry,
}
```

**关键操作对照：**

| 操作 | OCCT 方法 | Truck-bop 方法 |
|------|-----------|----------------|
| 注册形状 | `Init(shapes)` | `register_vertex_source(rank)`, `register_edge_source(rank)`, `register_face_source(rank)` |
| 存储干涉 | `AddInterf(interf)` | `push_vv_interference(interf)`, `push_ee_interference(interf)` 等 |
| Pave 排序去重 | 在 PaveFiller 中手动处理 | `normalize_edge_paves(edge_id)` 自动排序+合并近邻 |
| 构建 PaveBlock | 迭代器驱动 | `rebuild_pave_blocks_for_edge(edge_id)` 从 Pave 窗口自动生成 |

> **Checkpoint 2：** OCCT 的 BOPDS_DS 是一个巨型可变容器，Truck 的 BopDs 通过 Rust 所有权系统强制了更清晰的读写边界。

---

## 3. 算法流水线对比

### 3.1 OCCT 的 PaveFiller 流水线

这是 OCCT 布尔运算最核心的算法——**被拆成 11 个源文件**（`BOPAlgo_PaveFiller_1.cxx` 到 `_11.cxx`），每个文件对应一个阶段：

```
BOPAlgo_PaveFiller::Perform()
│
├── PerformVV()       ← _1.cxx: Vertex×Vertex 合并
├── PerformVE()       ← _2.cxx: Vertex×Edge 投影
├── PerformEE()       ← _3.cxx: Edge×Edge 交集 → 生成 Pave
├── PerformVF()       ← _4.cxx: Vertex×Face 投影
├── PerformEF()       ← _5.cxx: Edge×Face 交集 → 生成 Pave
├── PerformFF()       ← _6.cxx: Face×Face → 生成 Section Curve
├── MakePaveBlocks()  ← _7.cxx: 从 Pave 构建 PaveBlock
├── MakeSplitEdges()  ← _8.cxx: 按 PaveBlock 分割边
├── BuildDraftShells()← _9.cxx: 草稿壳体构建
├── ProcessDegenerated()← _10.cxx: 退化情况处理
└── Postprocess()     ← _11.cxx: 后处理与清理
```

**后续阶段（在 BOPAlgo_Builder 中）：**

```
BOPAlgo_Builder::PerformInternal()
│
├── FillImagesVertices()    ← Builder_1.cxx
├── FillImagesEdges()       ← Builder_1.cxx
├── FillImagesFaces()       ← Builder_2.cxx: 面分割+面分类
├── FillImagesContainers()  ← Builder_3.cxx
└── BuildResult()           ← Builder_4.cxx: 最终组装
```

### 3.2 Truck-bop 的流水线

Truck-bop 目前实现的流水线更精简，核心集中在三个文件：

**阶段 1：交集计算（`intersect/` 目录）**

```
intersect_vv() ← vv.rs
intersect_ve() ← ve.rs
intersect_ee() ← ee.rs
intersect_vf() ← vf.rs
intersect_ef() ← ef.rs
intersect_ff() ← ff.rs
```

**阶段 2：点分类（`pipeline.rs`）**

```rust
classify_point_in_solid(solid, point, tolerance)
│
├── 快速路径: classify_point_in_aabb()    ← 轴对齐包围盒优化
│   └── 仅对 6 面的轴对齐盒子有效
│
├── 通用路径: classify_point_by_ray_casting()
│   ├── boundary_proximity_check()        ← 先检查是否在边界上
│   ├── try_ray_classify()                ← 3 个方向的射线投射
│   │   ├── face_boundary_uv_hints()      ← UV 空间初始猜测
│   │   ├── ray_surface_newton()          ← Newton 迭代求交
│   │   └── uv_inside_face()             ← UV 空间内外判断
│   └── dedup_hits()                      ← 去重交点
│
└── 兜底路径: classify_point_by_nearest_face()
    └── 最近面法线判断
```

**阶段 3：裁剪与拓扑重建（`trim.rs` — 169KB 最大文件）**

```
build_trimming_loops()          ← 构建裁剪环
build_split_faces()             ← 面分割
classify_split_faces_against_operand() ← 面片分类
select_split_faces_for_boolean_op()    ← 根据布尔类型选取面片
merge_equivalent_vertices()     ← 合并等价顶点
sew_fragment_edges()            ← 缝合边
assemble_shells()               ← 壳体组装
build_solids_from_shells()      ← 构建实体
```

### 3.3 旧实现 — `truck-shapeops/transversal`

旧实现走完全不同的技术路线——**基于多边形三角化**：

```
and(solid_a, solid_b) / or(solid_a, solid_b)
│
├── intersection_curve    ← 计算两个 Shell 的交集曲线
├── loops_store           ← 将交集曲线组织为环
├── polyline_construction ← 多边线离散化
├── divide_face           ← 用交集曲线分割面
├── faces_classification  ← 射线法分类面（AND/OR/UNKNOWN）
└── integrate             ← 按布尔类型选面 → 组装 Shell → Solid
```

---

## 4. 三套方案的全景对比

| 维度 | OCCT (C++) | truck-bop (Rust 新) | truck-shapeops (Rust 旧) |
|------|------------|---------------------|--------------------------|
| **算法范式** | PaveFiller 参数化分段 | 移植 PaveFiller + 射线分类 | 多边形三角化 + 射线分类 |
| **核心数据结构** | BOPDS_DS（3000 行） | BopDs（500 行） | 无统一数据结构 |
| **交集精度** | 精确参数化 + Fuzzy 容差 | 精确参数化 + 独立容差 | 依赖三角化密度 |
| **交集类型** | VV/VE/EE/VF/EF/FF 全覆盖 | VV/VE/EE/VF/EF/FF 全覆盖 | 仅 Face×Face |
| **面分割** | PaveBlock → 分割边 → WireEdgeSet → 面重建 | TrimLoop → SplitFace → SewnPath → Shell | 交集曲线直接切割 |
| **面分类** | General Fuse + 面选择 | 射线投射 + 最近面法线兜底 | 射线投射 |
| **空间索引** | BoxTree（BOPTools） | broad_phase.rs（候选对生成） | 无 |
| **并行化** | TBB 并行 | 未并行（预留 Rayon） | 无 |
| **鲁棒性** | 30 年积累，Fuzzy 模式 | 微段标记、容差传播 | 基础容差 |
| **代码量** | ~30 万行 | ~22000 行 | ~3000 行 |
| **成熟度** | 生产级 | 开发中（stubs 未连通） | 可用（仅限横截相交） |
| **API** | `BRepAlgoAPI_Common/Cut/Fuse` | `common()/fuse()/cut()` (返回 NotImplemented) | `and()/or()` |

---

## 5. 深入关键设计决策

### 5.1 ID 系统 — 为什么 Truck 用 newtype？

OCCT 中，所有拓扑元素在 `BOPDS_DS` 中用 `Standard_Integer` 索引。这意味着编译器无法阻止你把一个 Edge 的索引传给需要 Face 索引的函数。

Truck-bop 采用 newtype 模式：

```rust
// bopds/ids.rs
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeId(pub u32);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FaceId(pub u32);
```

编译器会在**编译期**拒绝 `push_ee_interference(EEInterference { edge1: FaceId(0), ... })`——这在 OCCT 中只能靠代码审查发现。

### 5.2 点分类 — 三层递进策略

Truck-bop 的 `classify_point_in_solid()` 展示了精巧的分层设计：

```
Layer 1: AABB 快速路径
  └── 条件: Shell 恰好 6 面 + 轴对齐
  └── 复杂度: O(1) — 直接比较坐标

Layer 2: 参数化射线投射
  └── 先检查边界邻近性（OnBoundary 快速返回）
  └── 3 个非对齐方向射线投射
  └── Newton 迭代求 ray-surface 交点
  └── UV 空间内外判断（winding number）

Layer 3: 最近面法线兜底
  └── 当所有射线退化时（切面、边缘情况）
  └── 找最近面 → 用法线方向判断内外
```

对比 OCCT 的实现分散在 `BOPTools_AlgoTools::ComputeState()` 和 `IntTools_Context::IsPointInFace()` 中，逻辑更难追踪。

### 5.3 裁剪系统 — trim.rs 为何如此庞大？

`trim.rs` 的 169KB 包含了从交集结果到最终拓扑重建的**完整后半流水线**：

```
Section Curves → Trimming Loops → Split Faces → 分类 → 选取 → 合并顶点 → 缝合边 → 组装 Shell → 构建 Solid
```

这对应 OCCT 中 `BOPAlgo_Builder_2.cxx`（面构建） + `BOPAlgo_Builder_3.cxx`（容器构建） + `BOPAlgo_BuilderSolid.cxx` + `BOPAlgo_ShellSplitter.cxx` + 部分 `BOPTools_AlgoTools.cxx` 的功能总和。

OCCT 拆成了 5+ 个文件，Truck 集中在一个文件中——这反映了两种工程风格的取舍：
- **OCCT：** 按功能拆分，单文件职责清晰，但跨文件追踪数据流困难
- **Truck：** 按流水线集中，数据流清晰可见，但单文件过大

### 5.4 宽相位碰撞检测

**OCCT (`BOPDS_Iterator`)：** 使用 `BOPTools_BoxTree`（BVH 树）做空间索引，生成候选相交对。

**Truck-bop (`broad_phase.rs`)：** 实现了 `generate_candidate_pairs()` 和 `generate_candidate_pairs_from_bopds()`，基于包围盒过滤不可能相交的元素对，减少后续精确交集计算的次数。

---

## 6. 当前状态与差距分析

### 6.1 已完成

- **数据结构层（bopds）：** Pave、PaveBlock、InterferenceTable、CommonBlock、FaceInfo、ID 系统 — **完整移植**
- **交集函数签名：** VV/VE/EE/VF/EF/FF 函数已定义
- **点分类：** 三层递进策略已实现且有完整测试
- **裁剪系统：** trimming loop 构建、split face 生成、面分类、边缝合、壳体组装 — **核心逻辑已实现**
- **宽相位：** 候选对生成已实现

### 6.2 未完成（API 层返回 NotImplemented）

```rust
pub fn common<C, S>(...) -> Result<Solid, BopError> {
    Err(BopError::NotImplemented("common"))  // ← 尚未连通
}
```

`lib.rs` 中的 `common()`/`fuse()`/`cut()`/`section()` 四个高层 API 全部返回 `NotImplemented`——说明内部组件已基本就绪，但**端到端流水线尚未串联**。

### 6.3 OCCT 有而 Truck-bop 尚缺的能力

| 能力 | OCCT 实现 | Truck-bop 状态 |
|------|-----------|----------------|
| Fuzzy 容差模式 | `BOPAlgo_Options::SetFuzzyValue()` | `BopOptions` 有基础容差，无 Fuzzy |
| Gluing 优化 | `BOPAlgo_GlueEnum` | 未实现 |
| 退化边/面处理 | `PaveFiller_10.cxx` | `unsplittable` 标记是初步方案 |
| 修改历史追踪 | `BOPAlgo_BuilderShape::History()` | 无 |
| 自交检测 | `BOPAlgo_CheckerSI` | 无 |
| 并行处理 | TBB | 未并行 |
| 分割器 | `BOPAlgo_Splitter` | 无 |
| CellsBuilder | `BOPAlgo_CellsBuilder` | 无 |
| RemoveFeatures | `BOPAlgo_RemoveFeatures` | 无 |

---

## 7. 总结与推荐学习路径

### 7.1 如果你想理解布尔运算原理

1. **从 Truck-bop 的数据结构入手** — `bopds/pave.rs` → `pave_block.rs` → `interference.rs` → `mod.rs`，Rust 代码比 C++ 更容易理解数据关系
2. **看 `pipeline.rs` 的点分类** — 精巧的三层设计是理解"面分类"的最佳入口
3. **对照 OCCT 的 `OCCT_to_Truck_Boolean_Migration_Plan.md`** — 它本身就是两个系统的架构映射文档

### 7.2 如果你想贡献 Truck-bop

最高优先级是**串联端到端流水线**：让 `lib.rs` 中的 `fuse()`/`common()`/`cut()` 调用 `intersect/*` → `trim.rs` → 返回结果 `Solid`。

### 7.3 如果你想深入 OCCT

从 `BRepAlgoAPI_Common.cxx` 开始，追踪调用链：
```
BRepAlgoAPI_Common → BOPAlgo_BOP → BOPAlgo_Builder → BOPAlgo_PaveFiller
```
PaveFiller 的 11 个子文件按编号顺序阅读即可。

---

**文档版本：** v1.0
**生成日期：** 2026-04-09
**基于代码快照：** truck-new (2026-04-07), OCCT (2026-03-15)
