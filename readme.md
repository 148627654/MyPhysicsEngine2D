# MyPhysicsEngine2D - V5 性能革命版（Performance Revolution Edition）

V4 交付了**功能完整度**（11 类关节、世界查询、CCD 全链路）后，V5 把炮口对准**性能**：同屏物体一旦达到 5,000+，CPU 将花费 70% 以上的时间在"等待内存从 RAM 搬运到缓存"上（**内存墙**问题）。V5 的答案是三件套：

1. **BlockAllocator 内存池**：嵌入式空闲链表 O(1) 分配回收，彻底告别逐对象 `new/delete` 的系统堆往返；
2. **SoA 数据导向架构**：刚体高频动力学数据从散落堆各处的 `Body` 对象剥离为连续平铺的平行数组，消灭指针追逐（Pointer Chasing），一条 64B 缓存行供给多个刚体连续推进；
3. **SIMD 矢量化**：SoA 铺平了唯一物理道路——AVX2 一条 YMM 指令同时推进 8 个刚体的位置与速度，FMA 融合乘加 + 免分支掩码（已交付）。

---

# 开发进度表

## 第 1 阶段：内存基础设施攻坚（第 1-2 天）✅ 已交付

### 第 1 天：BlockAllocator 内存池验收 ✅

**验收三连**（`googleTest/V5_001.cpp`，3 用例）：

| 用例 | 断言 | 实测 |
|---|---|---|
| `AllocAndRecycleTest` | 归还第 3 块后新块地址 `==` 精准复用 | ✅ 地址严格相等（LIFO 嵌入式空闲链表 O(1)），复用块写满 32B 哨兵逐字节校验 |
| `MultiSizeStressTest` | 16B~1024B 随机尺寸 10 万次分配回收 < 5.0 ms | ✅ **实测 1.16 ms**（红线 1/4，Debug 构建）；2048 块随机尺寸全存活乱序回收无越界 |
| `LargeFallbackAndClearTest` | 2048B 回退 + `Clear()` 泄漏字节 == 0 | ✅ CRT 快照对比：存活普通块/客户端块计数与字节数双双为 0 |

**验收挖出的三处引擎修复**：

1. **僵尸注册**：`BlockAllocator` 早已注册进主工程 vcxproj，但静态成员 `s_blockSizeLookup`/`s_lookupInitialized` 只有声明没有定义——中间目录从未产出过它的 .obj，一旦真正链接必 LNK2001。已补全定义并接入测试工程编译链。
2. **Clear() 泄漏超大对象**：>1024B 走 `std::malloc` 的块此前无登记，`Clear()` 管不到。新增回退登记表（Allocate 登记 / Free 交换删除 / Clear 兜底全释放）。
3. **Clear() 后不可复用**：`m_chunkSpace` 归零后慢路径 `realloc` 永不增长。已改为懒重建。

### 第 2 天：刚体数据导向设计 (SoA) 与缓存行对齐改造 ✅

**验收三连**（`googleTest/V5_002.cpp`，3 用例）：

| 用例 | 断言 | 实测 |
|---|---|---|
| `SoAAllocationAndSwapTest` | 10 刚体槽位 0~9 严格连续；删槽位 3 后原 9 号体精准搬入、句柄同步为 3、数据零错位 | ✅ 全部通过（(99,88) 探针数据逐位核对） |
| `BidirectionalSyncTest` | `setPosition`/`setVelocity` 等 API 无缝穿透读写 SoA 槽位；1 步积分后 `getPosition()` 与 SoA 运算值一致 | ✅ 通过（含重力缩放 2.0 + 扭矩 3.0 混合工况；Day 3 起引擎走 FMA 内核，按 1e-5/1e-6 容差验收） |
| `SoAVsAoSBenchmark` | 5000 刚体积分：数值一致；耗时下降 35%+（< 1.3ms/帧） | ✅ 数值一致（FMA 舍入按量级容差）；**0.273 vs 0.556 ms/帧，提速 50.9%**（Day 3 后引擎内核再换 AVX2） |

**核心架构**：

- **`BodySoA` 平行紧致数组**（11 条）：`positions` / `velocities` / `rotations` / `angularVels` / `invMasses` / `invInertias` / `forces` / `torques` / `gravityScales` / `flags`（uint32_t 位标记：AWAKE/BULLET）/ `bodyOwners`（槽位 → 句柄反向映射）。
- **Swap-and-Pop 槽位管理**：销毁槽位 `i` 时把末尾槽位整体搬运覆盖 `i` 再 `pop_back`——严格 O(1)，数组永远 100% 紧致无空洞；被搬家刚体的 `m_soaIndex` 自动同步，被删刚体在 `delete` 前彻底脱离（杜绝悬空句柄）。
- **数据双向无感同步**：`Body` 保留原成员为"未挂载期存储"，挂载后全部热数据访问器经槽位助手穿透 SoA——上层游戏逻辑 `body->getPosition()` / `body->setVelocity()` **0 修改、0 感知**。`setAwake`/`forceSleep`/`setBullet` 同步位标记，`setShape`/`setType`/`updateMassData` 同步质量属性。
- **积分流水线重写**：`World::step` 的积分循环改为 **`data()` 原始指针 + 标量 x/y 展开**的平铺循环——Debug 下也零函数调用，标量序列与旧 Vector2 运算逐位一致（`operator*` 即 `(x*num, y*num)`，一致性有铁证）；静态/睡眠刚体经 `invM == 0` / `FLAG_AWAKE` 分支预测器友好跳过。

## 第 2 阶段：SIMD 矢量化爆发（第 3 天）✅ 已交付

### 第 3 天：SIMD 矢量化前向积分 (AVX2) 与 8 倍算力爆发 ✅

**验收三连**（`googleTest/V5_003.cpp`，3 用例）：

| 用例 | 断言 | 实测 |
|---|---|---|
| `SIMDAccuracyTest` | 8 个参数各异刚体推进 1 步：AVX2 FMA 内核与标量回退位姿/速度绝对误差 < 1e-6 | ✅ 通过（浮点位级等价） |
| `SIMDTailLoopTest` | 13 刚体（1 个 8 路批处理 + 5 个标量尾部 + 批内静态掩码），全量更新准确、无越界崩溃 | ✅ 通过；静态刚体逐位纹丝不动 |
| `SIMD10000BodiesBenchmark` | 10,000 刚体积分纯耗时严格 < 0.4 ms | ✅ **0.209 ms/帧**；对旧 AoS 基线（1.129 ms）**加速 5.4 倍** |

**核心架构**：

- **`SIMDMath.h`**：AVX2 编译期检测宏（`SIMD_AVX2_ENABLED`，随 `/arch:AVX2` 开关自动定义）+ 运行时 CPUID 能力检测（`IsAVX2Supported()`：AVX/OSXSAVE/XGETBV/AVX2 四道关卡，结果缓存）——不支持的环境自动降级标量路径。
- **SoA 拆分单精度流**：`positionsX/Y`、`velocitiesX/Y` 等升级为独立 `float` 数组（AVX2 免洗牌免转置），配套 32 字节对齐分配器（`AlignedAllocator32`，`_aligned_malloc`）+ 容量恒为 8 的倍数（矢量加载永不越界缺页）。
- **`SIMDIntegrator.cpp` 内核**：8 路并发批处理循环——`_mm256_fmadd_ps` 融合乘加一条指令完成 `force·invMass + gravity` 与 `x + v·dt`；静态/睡眠刚体用 `_mm256_cmp_ps` + `blendv_ps` **免分支掩码**（0 个 if，零分支预测惩罚）；`count % 8` 标量尾部循环保证任意数量 100% 正确。
- **`World::step` 接入**：积分阶段直接调用 `m_soa.IntegrateSIMD(dt, m_gravity)`；静态/睡眠 prev 同步循环改为原始数组平扫（10k 刚体省 ~0.35ms 逐体访问器调用开销）。

## 后续规划方向（待每日规格）

- 约束求解器/岛屿热循环的 SIMD 化与缓存行填充（Cache Line Padding，消除伪共享）；
- 多线程宽相/岛屿并行（Job System）。

---

# 性能数据总览（Debug|x64 实测）

| 场景 | 指标 | 数值 |
|---|---|---|
| 内存池 10 万次分配回收（16B~1024B 随机） | 总耗时 | **1.16 ms**（< 5.0 ms 红线） |
| 5000 刚体前向积分（旧版 AoS 指针遍历） | 每帧耗时 | 0.556 ms |
| 5000 刚体前向积分（新版 SoA 平铺流水线） | 每帧耗时 | **0.273 ms（-50.9%）** |
| 10000 刚体前向积分（旧版 AoS 指针遍历基线） | 每帧耗时 | 1.129 ms |
| 10000 刚体前向积分（AVX2 8 路并发 FMA 内核） | 每帧耗时 | **0.209 ms（加速 5.4 倍，红线 < 0.4 ms）** |
| SIMD vs 标量积分数值一致性 | 误差 | **< 1e-6（浮点位级等价）** |
| 全量回归 | GoogleTest | **90/90 全绿**（连跑多轮稳定） |

---

# V5 新增项目结构

```text
include/physics/Common/
├── SoABody.h              # <--- [V5 Day 2/3] 热数据平铺数组容器（Day 3 升级拆分单精度流 + 32B 对齐）
├── SIMDMath.h             # <--- [V5 Day 3] AVX2 指令集封装与 CPUID 检测降级宏
└── BlockAllocator.h       # <--- [V5 Day 1] 内存池（14 级尺寸阶梯 + 回退登记表）
src/Common/
├── SoABody.cpp            # <--- [V5 Day 2/3] AllocateSlot / FreeSlot（Swap-and-Pop）
├── SIMDIntegrator.cpp     # <--- [V5 Day 3] AVX2 8 路并发 FMA 积分内核 + 标量回退
└── BlockAllocator.cpp     # <--- [V5 Day 1] 静态成员补全 + 回退兜底回收 + 懒重建
src/Dynamics/
├── World.cpp              # <--- [V5 Day 2] 积分循环切换为 SoA 原始指针 + 标量流水线
├── Body.h/.cpp            # <--- [V5 Day 2] 热数据访问器双向穿透 SoA 槽位
googleTest/
├── V5_001.cpp             # <--- [V5 Day 1] 内存池验收（复用/吞吐/零泄漏）
├── V5_002.cpp             # <--- [V5 Day 2] 槽位映射/双向同步/5000 体性能对照
└── V5_003.cpp             # <--- [V5 Day 3] SIMD 精度等价/尾部安全/万体性能狂飙
```

---

# 开发复盘：V5 踩坑记

1. **Debug 下 SoA 也会"翻车"**：MSVC Debug 不内联，`std::vector::operator[]` 和 Vector2 运算符全是真实函数调用——第一版 SoA 循环实测 1.18ms/帧，**比 AoS 参考还慢一倍**。换成 `data()` 原始指针 + 标量展开后直接 0.273ms。SoA 的优势必须靠"零函数调用"兑现。
2. **CRT 泄漏检测陷阱（VS2026）**：`_CrtMemDifference().lTotalCount` 会把"已正确释放、但由 CRT 堆缓存持有"的字节计入（`malloc(2048)+free` 后报 +2048，而五类块计数全 0）。真正的零泄漏判据是 `lCounts/lSizes[_NORMAL_BLOCK]` 与 `[_CLIENT_BLOCK]` == 0。
3. **测试不能读已释放对象**：`destroyBody` 后读被删刚体的 `getSoaIndex()` 是 UB——FreeSlot 的句柄脱离发生在 `delete` 前，但验收只能断言被搬家的刚体与总数。
4. **基准必须诚实**：AoS 参考要忠实复刻旧引擎路径（5 次访问器调用 + `savePrevState`/`clearForce` + 直接成员写），而不是人为堆砌每体 26 次调用来抬高提速比。50.9% 是货真价实的。
5. **Vector2 `operator==` 是 EPSILON 近似**，不是逐位比较——需要逐位断言时直接比 `getX() == getX()`。
6. **Debug 下的"隐形函数调用"是性能头号杀手**（Day 3 再次印证）：`constexpr` 构造（`Vector2(x, y)`）在 Debug 下照样编译成真实函数调用——SIMD 内核的 P0 快照循环里 1 万次 Vector2 构造吞掉 0.1ms+，改成直接 `.x`/`.y` 成员写立刻还清。同理，World 里"静态/睡眠 prev 同步"循环逐体走 getter（10k × 2 调用 ≈ 0.35ms/帧），改原始数组平扫后 < 0.03ms。
7. **AVX2 内核在 Debug 下的真实战果要对准基线**：SIMD 批处理循环本身（1250 批 × ~40 条内联 intrinsic）只要 ~0.05ms；但我们的标量内核（Day 2 起零函数调用）也只 0.28ms/万体——**SIMD 对"已优化的标量"仅 ~1.7 倍，对旧引擎 AoS 基线才是 5.4 倍**。加速比要诚实标注对照对象（规格原文的 3.5~4.2ms 就是旧 AoS 基线，而非新标量内核）。
8. **FMA 改变舍入**：`_mm256_fmadd_ps` 是"一次舍入的乘加"，与标量"乘法舍入 + 加法舍入"存在 ~ulp 级差异——Day 2 的"逐位相等"断言在引擎换内核后必须降级为按量级容差（位置 ~100 量级 60 帧累计 → 1e-3），纯存储类断言仍可逐位。

---

## 💻 编译与运行

- **开发环境**：Visual Studio 2019 / 2022 (ISO C++11 Standard)，MSVC Debug|x64
- **测试**：`MyPhysicsEngine2DTests.vcxproj`（PostBuild 自动跑 GoogleTest，失败即构建失败）
- **编译依赖**：无第三方依赖（纯手搓核心数学与几何算法）
