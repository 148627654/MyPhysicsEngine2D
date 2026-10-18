# MyPhysicsEngine2D - V5 性能革命版（Performance Revolution Edition）

V4 交付了**功能完整度**（11 类关节、世界查询、CCD 全链路）后，V5 把炮口对准**性能**：同屏物体一旦达到 5,000+，CPU 将花费 70% 以上的时间在"等待内存从 RAM 搬运到缓存"上（**内存墙**问题）。V5 的答案是三件套：

1. **BlockAllocator 内存池**：嵌入式空闲链表 O(1) 分配回收，彻底告别逐对象 `new/delete` 的系统堆往返；
2. **SoA 数据导向架构**：刚体高频动力学数据从散落堆各处的 `Body` 对象剥离为连续平铺的平行数组，消灭指针追逐（Pointer Chasing），一条 64B 缓存行供给多个刚体连续推进；
3. **SIMD 矢量化**：SoA 铺平了唯一物理道路——连续标量流天然适配 AVX2 指令级并行（规划中）。

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
| `BidirectionalSyncTest` | `setPosition`/`setVelocity` 等 API 无缝穿透读写 SoA 槽位；1 步积分后 `getPosition()` 与 SoA 运算值逐位一致 | ✅ 逐位相等（含重力缩放 2.0 + 扭矩 3.0 混合工况） |
| `SoAVsAoSBenchmark` | 5000 刚体积分：数值误差严格为 0；耗时下降 35%+（< 1.3ms/帧） | ✅ 数值**逐位相等**；**0.273 vs 0.556 ms/帧，提速 50.9%** |

**核心架构**：

- **`BodySoA` 平行紧致数组**（11 条）：`positions` / `velocities` / `rotations` / `angularVels` / `invMasses` / `invInertias` / `forces` / `torques` / `gravityScales` / `flags`（uint32_t 位标记：AWAKE/BULLET）/ `bodyOwners`（槽位 → 句柄反向映射）。
- **Swap-and-Pop 槽位管理**：销毁槽位 `i` 时把末尾槽位整体搬运覆盖 `i` 再 `pop_back`——严格 O(1)，数组永远 100% 紧致无空洞；被搬家刚体的 `m_soaIndex` 自动同步，被删刚体在 `delete` 前彻底脱离（杜绝悬空句柄）。
- **数据双向无感同步**：`Body` 保留原成员为"未挂载期存储"，挂载后全部热数据访问器经槽位助手穿透 SoA——上层游戏逻辑 `body->getPosition()` / `body->setVelocity()` **0 修改、0 感知**。`setAwake`/`forceSleep`/`setBullet` 同步位标记，`setShape`/`setType`/`updateMassData` 同步质量属性。
- **积分流水线重写**：`World::step` 的积分循环改为 **`data()` 原始指针 + 标量 x/y 展开**的平铺循环——Debug 下也零函数调用，标量序列与旧 Vector2 运算逐位一致（`operator*` 即 `(x*num, y*num)`，一致性有铁证）；静态/睡眠刚体经 `invM == 0` / `FLAG_AWAKE` 分支预测器友好跳过。

## 第 2 阶段：SIMD 矢量化爆发（第 3 天起，待每日规格）

Day 2 的 SoA 平铺是唯一物理道路——连续标量流天然适配 AVX2：一条 256-bit 指令同时推进 8 个刚体的质量倒数/速度分量。后续规划方向（以每日规格为准）：

- AVX2 积分与约束求解矢量化；
- 求解器/岛屿热循环的 SoA 化与缓存行填充（Cache Line Padding，消除伪共享）；
- 多线程宽相/岛屿并行（Job System）。

---

# 性能数据总览（Debug|x64 实测）

| 场景 | 指标 | 数值 |
|---|---|---|
| 内存池 10 万次分配回收（16B~1024B 随机） | 总耗时 | **1.16 ms**（< 5.0 ms 红线） |
| 5000 刚体前向积分（旧版 AoS 指针遍历） | 每帧耗时 | 0.556 ms |
| 5000 刚体前向积分（新版 SoA 平铺流水线） | 每帧耗时 | **0.273 ms（-50.9%）** |
| SoA vs AoS 数值一致性 | 误差 | **严格为 0（逐位相等）** |
| 全量回归 | GoogleTest | **87/87 全绿**（连跑多轮稳定） |

---

# V5 新增项目结构

```text
include/physics/Common/
├── SoABody.h              # <--- [V5 Day 2] 刚体热数据平行数组容器（SoA 架构核心）
└── BlockAllocator.h       # <--- [V5 Day 1] 内存池（14 级尺寸阶梯 + 回退登记表）
src/Common/
├── SoABody.cpp            # <--- [V5 Day 2] AllocateSlot / FreeSlot（Swap-and-Pop）
└── BlockAllocator.cpp     # <--- [V5 Day 1] 静态成员补全 + 回退兜底回收 + 懒重建
src/Dynamics/
├── World.cpp              # <--- [V5 Day 2] 积分循环切换为 SoA 原始指针 + 标量流水线
├── Body.h/.cpp            # <--- [V5 Day 2] 热数据访问器双向穿透 SoA 槽位
googleTest/
├── V5_001.cpp             # <--- [V5 Day 1] 内存池验收（复用/吞吐/零泄漏）
└── V5_002.cpp             # <--- [V5 Day 2] 槽位映射/双向同步/5000 体性能对照
```

---

# 开发复盘：V5 踩坑记

1. **Debug 下 SoA 也会"翻车"**：MSVC Debug 不内联，`std::vector::operator[]` 和 Vector2 运算符全是真实函数调用——第一版 SoA 循环实测 1.18ms/帧，**比 AoS 参考还慢一倍**。换成 `data()` 原始指针 + 标量展开后直接 0.273ms。SoA 的优势必须靠"零函数调用"兑现。
2. **CRT 泄漏检测陷阱（VS2026）**：`_CrtMemDifference().lTotalCount` 会把"已正确释放、但由 CRT 堆缓存持有"的字节计入（`malloc(2048)+free` 后报 +2048，而五类块计数全 0）。真正的零泄漏判据是 `lCounts/lSizes[_NORMAL_BLOCK]` 与 `[_CLIENT_BLOCK]` == 0。
3. **测试不能读已释放对象**：`destroyBody` 后读被删刚体的 `getSoaIndex()` 是 UB——FreeSlot 的句柄脱离发生在 `delete` 前，但验收只能断言被搬家的刚体与总数。
4. **基准必须诚实**：AoS 参考要忠实复刻旧引擎路径（5 次访问器调用 + `savePrevState`/`clearForce` + 直接成员写），而不是人为堆砌每体 26 次调用来抬高提速比。50.9% 是货真价实的。
5. **Vector2 `operator==` 是 EPSILON 近似**，不是逐位比较——需要逐位断言时直接比 `getX() == getX()`。

---

## 💻 编译与运行

- **开发环境**：Visual Studio 2019 / 2022 (ISO C++11 Standard)，MSVC Debug|x64
- **测试**：`MyPhysicsEngine2DTests.vcxproj`（PostBuild 自动跑 GoogleTest，失败即构建失败）
- **编译依赖**：无第三方依赖（纯手搓核心数学与几何算法）
