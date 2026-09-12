# MyPhysicsEngine2D - V3

一个基于 C++11 构建的高仿真 2D 刚体物理引擎。在 V1 稳健的离散动力学与 V2 工业级性能架构（动态 AABB 树、休眠岛屿、CCD）的基础上，**V3 致力于功能特性的全面扩展与高阶约束力学**。

## 📌 项目愿景
V3 阶段的目标是将引擎从“纯碰撞计算库”升级为“游戏引擎级全功能物理套件”。通过几何体扩充、高级多自由度关节解算、材质属性解耦以及碰撞事件总线，让物理世界不仅“跑得快、算得准”，更具备“丰富的表现力与高阶游戏交互能力”。
- **高级约束力学**：基于拉格朗日乘子与 Sequential Impulses 求解器，支持 Distance、Spring（软约束阻尼）与 Revolute 铰链关节。
- **复杂几何支持**：引入胶囊体（Capsule）与通用凸多边形（Convex Polygon），实现基于 Sutherland-Hodgman 算法的高精度接触裁剪。
- **材质与传感器体系**：物理材质完全解耦，支持动静摩擦与弹性恢复多模式混合，原生集成 Trigger 穿透触发器。
- **事件总线机制**：构建基于帧状态缓存的碰撞生命周期状态机，开箱即用支持 `OnCollisionEnter / Stay / Exit`。

---

## 🛠 项目结构 (V3 更新)
```text
MyPhysicsEngine2D/
├── include/
│   └── physics/
│       ├── Collision/
│       │   ├── Shapes/
│       │   │   ├── Shape.h        # <--- [V3] 增加 Material/isTrigger/UpdateMassData
│       │   │   ├── Box.h          # <--- [V3] 实现基于密度的质量/转动惯量自适应
│       │   │   ├── Circle.h       # <--- [V3] 接入统一材质与质量更新接口
│       │   │   ├── Capsule.h      # <--- [V3 待开发] 胶囊体定义
│       │   │   └── Polygon.h      # <--- [V3 待开发] 任意凸多边形
│       │   └── ...
│       ├── Dynamics/
│       │   ├── Material.h         # <--- [V3] 物理材质与 CombineMode 仲裁器
│       │   ├── Joints/            # <--- [V3 待开发] 多自由度约束关节模块
│       │   └── ContactSolver.h    # <--- [V3] 解算器接入 Trigger 旁路判断
│       └── Events/                # <--- [V3 待开发] 碰撞事件调度与监听总线
├── src/
│   ├── Collision/
│   │   └── Shapes/
│   │       ├── Box.cpp            # <--- [V3] 矩形几何与惯量推导
│   │       └── ...
│   ├── Dynamics/
│   │   └── Material.cpp           # <--- [V3] 摩擦/弹性恢复混合数学实现
│   └── ...
└── README.md
```

---

## 📅 进度跟踪 (V3 14天挑战)

### 第 1 阶段：材质属性与复杂几何体扩展
- [x] **Day 01: 材质系统 (Material) 与 触发器 (Trigger) 架构**
- [x] **Day 02: 胶囊体碰撞 (Capsule Collider)**
- [ ] **Day 03: 通用凸多边形表示与惯性属性 (Convex Polygon)**
- [ ] **Day 04: 多边形接触流形裁剪 (Sutherland-Hodgman Clipping)**

### 第 2 阶段：生命周期与事件系统
- [ ] **Day 05: 接触状态缓存与生命周期判定 (Contact Cache)**
- [ ] **Day 06: 观察者模式与事件总线 (Event System)**

### 第 3 阶段：约束与关节系统 (Joints & Constraints)
- [ ] **Day 07: 约束求解器骨架与 Baumgarte 稳定化**
- [ ] **Day 08: 距离关节 (Distance Joint)**
- [ ] **Day 09: 软约束弹簧阻尼关节 (Spring Joint)**
- [ ] **Day 10: 铰链/旋转关节 (Revolute Joint) 基础**
- [ ] **Day 11: 关节马达与限位约束 (Motor & Angle Limits)**

### 第 4 阶段：复合场景验证与工程收尾
- [ ] **Day 12: 复杂物理用例构建 (Ragdoll, Bridge & Car Demo)**
- [ ] **Day 13: 闭环约束能量漂移与单元测试**
- [ ] **Day 14: 接口统一、数据导出与发布**

---

## 🚀 Day 01 进展：材质系统与触发器架构

### 1. 技术核心：几何、材质与力学的“三权分立”
在 V1/V2 阶段，物体的质量通常是手动填写的标量，摩擦力和弹性分散各处。在 V3 中，我们重构了底层逻辑，确立了清晰的职责边界：
- **Shape 负责几何**：只关心形状、尺寸、局部顶点以及纯几何积分（如面积 $A$、外接半径）。
- **Material 负责物理性质**：包含密度（Density）、弹性恢复系数（Restitution）以及静摩擦与动摩擦系数。
- **Body 负责力学表现**：汇集来自 Shape 的几何尺寸与 Material 的密度，自动解算刚体的真实质量（$m = A \cdot \rho$）与转动惯量（$I$）。

### 2. 核心机制解析

#### A. 材质混合仲裁 (Material Combine Strategy)
现实世界中不同物体表面接触时，接触面的摩擦与弹性不是单方面决定的。引擎支持四种标准仲裁策略：
- **弹性混合 (`restitutionCombine`)**：默认采用 **`Maximum`**，确保高弹性网球砸向吸能地面依然能显著反弹：
  $$e_{\text{combined}} = \max(e_A, e_B)$$
- **摩擦混合 (`frictionCombine`)**：默认采用 **`Multiply`（几何平均）**，符合经典库仑定律实验结论（冰面接触粗糙面摩擦急剧降低）：
  $$\mu_{\text{combined}} = \sqrt{\mu_A \cdot \mu_B}$$

#### B. 触发器旁路机制 (Trigger / Sensor Pipeline)
游戏开发中大量场景只需“感知重叠”而非“阻挡推开”（如检查点、金币拾取、陷阱区域）：
- 将 `Shape::isTrigger` 设为 `true`。
- 宽相 AABB 树与窄相流形计算照常执行，完整生成法线与穿透数据。
- **解算器切断**：在 `ContactSolver` 阶段直接拦截，跳过法向推开冲量与切向摩擦力冲量计算。物体如“幽灵”般穿过触发器，同时零损耗保留物理动能。

---

### 3. 开发复盘：那些让我们“翻车”的 Bug

#### **Bug A: 抽象基类无法实例化的编译死锁**
- **现象**：在 `Shape` 中引入了 `virtual void UpdateMassData() = 0;` 纯虚函数后，`new Box(...)` 直接触发 MSVC 编译错误 `C2259: cannot instantiate abstract class`。
- **根因**：派生类 `Box` 中未重写基类纯虚接口，导致子类隐式退化为抽象类。
- **解决**：在 `Box` 与 `Circle` 中显式添加 `void UpdateMassData() override;`，基于自身几何形状和绑定的 `material.density` 实现完整的物理参数自适应。

#### **Bug B: 传感器物体与解算器“暗度陈仓”**
- **现象**：将物体设为 `isTrigger = true` 后，测试物体在下落接触瞬间依然发生了轻微弹跳并丢失了垂直动能。
- **根因**：虽然在速度解算器（Velocity Constraint）中增加了 `isTrigger` 拦截，但忘记在 **位置修正器（Position Correction / Baumgarte）** 中拦截，导致重叠穿透修正把物体强行推了出去。
- **解决**：在 `Contact` 流形中注入 `isSensor` 标志位，解算器在执行冲量与穿透修正的全流程对 Sensor 接触对实施无差别放行。

---

### 4. 如何验证
运行 `tests/MaterialTests.cpp`。当前已通过以下综合校验：
- ✅ **触发器无损穿透验证**：刚体穿过 Trigger 传感器时向下速度高达 `-10.78 m/s`，未损失任何动量；随后撞击真实地表并平稳休眠（`asleep=1`），证明 Trigger 旁路机制与 V2 休眠系统无缝协同。
- ✅ **数值混合测试 (CombineMode)**：验证了 `Multiply`、`Maximum` 等模式在不同浮点边界下的运算确定性与裁剪合法性。
- ✅ **密度动态回写验证 (Write-back)**：修改材质密度后，刚体质量与其逆质量完成精准等比放大，转动惯量同步更新。

**Day 01 运行快照：**
```text
[INFO] >>> Starting V3 001: Material & Trigger Test...
[INFO] Trigger: passed=1 vy_at_pass=-10.779996 finalY=-0.529077 onGround=1 asleep=1
[INFO] CombineMode: PASS
[INFO] density write-back & delegate: PASS
[INFO] >>> ALL TESTS PASSED <<<
```
这里是为你量身编写的 **V3 - Day 02 任务总结（README 增补内容）**。

你可以直接将其复制并追加到你的 `README.md` 中（同时将顶部进度表中的 `Day 02` 勾选为 `[x]`）：

---

## 🚀 Day 02 进展：胶囊体碰撞 (Capsule Collider) 与线段退化几何学

### 1. 技术核心：闵可夫斯基几何退化与质量积分
胶囊体（Capsule）是现代物理引擎刻画角色控制器（Character Controller）和布娃娃（Ragdoll）的基石。在数学上，胶囊体本质上是**骨架线段与半径为 $r$ 的圆盘的闵可夫斯基和（Minkowski Sum）**。

#### A. 几何积分与转动惯量合成
胶囊体由一个矩形柱体（$2r \times L$）和两端半圆（等价于一个半径为 $r$ 的完整圆盘）组成：
- **精确面积**：
  $$Area = 2rL + \pi r^2$$
  *(在测试用例中，$r=1, L=4$，实测面积精准匹配预期值 $11.141592$)*
- **转动惯量推导（平行轴定理）**：
  将中央矩形与两端半球拆分积分：
  $$I_{\text{rect}} = \frac{1}{12} m_{\text{rect}} ((2r)^2 + L^2)$$
  $$I_{\text{caps}} = m_{\text{caps}} \left( \frac{1}{2}r^2 + \left(\frac{L}{2}\right)^2 \right)$$
  $$I_{\text{total}} = I_{\text{rect}} + I_{\text{caps}}$$
  在 `Capsule::ComputeMass` 中根据当前材质密度 $\rho$ 实现了质量与转动惯量的自适应更新。

---

### 2. 核心碰撞检测算法深度实现详解

由于胶囊体的几何本质是“线段 + 半径”，所有的复杂碰撞都可以**退化为线段与其他几何特征的距离极值问题**：

#### ① `CapsuleVsCircle`：点到线段的几何退化
```cpp
static bool CapsuleVsCircle(Manifold* m, Body* capsuleBody, Body* circleBody);
```
*   **实现原理**：
    1.  **世界坐标转换**：提取胶囊体世界骨架线段的两个端点 $A, B$。
    2.  **最近投影点求解**：调用 `ClosestPointOnSegment`，求出圆心 $C$ 到线段 $AB$ 的正交投影最近点 $P$：
        $$t = \text{clamp}\left(\frac{(C - A) \cdot (B - A)}{\|B - A\|^2}, 0, 1\right), \quad P = A + t(B - A)$$
    3.  **虚拟圆对撞**：将碰撞问题等价退化为——位于点 $P$、半径为 $r_{\text{cap}}$ 的虚拟圆，与目标圆 $C$（半径 $r_{\text{circ}}$）的圆-圆相交测试。
    4.  **流形生成**：若距离 $d < r_{\text{cap}} + r_{\text{circ}}$，则法线 $\mathbf{n} = (C - P) / d$，穿透深度为半径和减去中心距。
*   **物理表现**：测试中小球垂直砸向平躺的胶囊体，反弹时横向位移偏差 `dx = 0.000000`，最大弹起速度达 $7.02\,\text{m/s}$，证明法线计算无横向漂移误差。

---

#### ② `CapsuleVsCapsule`：两线段间最短距离算法 (Christer Ericson 解法)
```cpp
static bool CapsuleVsCapsule(Manifold* m, Body* bodyA, Body* bodyB);
```
*   **实现原理**：
    两个胶囊体的碰撞，等价于求解两条三维/二维线段 $S_1(s)$ 与 $S_2(t)$ 之间的**最近点对 $(P_A, P_B)$**：
    $$S_1(s) = A_1 + s \mathbf{d}_1 \quad (s \in [0, 1]), \qquad S_2(t) = A_2 + t \mathbf{d}_2 \quad (t \in [0, 1])$$
    1.  **参数空间二次优化**：构建距离函数 $Q(s, t) = \|S_1(s) - S_2(t)\|^2$，对其求偏导并解二元一次方程组，求出未裁剪参数：
        $$\text{denom} = (\mathbf{d}_1 \cdot \mathbf{d}_1)(\mathbf{d}_2 \cdot \mathbf{d}_2) - (\mathbf{d}_1 \cdot \mathbf{d}_2)^2$$
    2.  **区域裁剪与平行防御**：当 $\text{denom} \approx 0$（两线段平行）时强制锁定 $s=0$；随后将未夹紧的 $s, t$ 严格截断至边界区间 $[0, 1]$ 内，得到绝对最近的 $P_A, P_B$。
    3.  **流形生成**：以点对连线为法向 $\mathbf{n} = (P_B - P_A) / \|P_B - P_A\|$，穿透量为 $(r_A + r_B) - \|P_B - P_A\|$。
*   **物理表现**：在十字交叉碰撞测试中，接触点法线精准输出为 `(0.000000, 1.000000)`，生成单点接触并将两刚体稳定支撑在 $Y=3.97$ 的平衡位置。

---

#### ③ `CapsuleVsBox`：局部空间 6 候选点极值采样与深穿透保护
```cpp
static bool CapsuleVsBox(Manifold* m, Body* capsuleBody, Body* boxBody);
```
*   **实现原理**：
    带任意朝向的 Box 与 Capsule 求交通常需使用复杂的 GJK 算法。我们采用了更为高效优雅的**“局部空间极值采样法”**：
    1.  **局部逆变换**：将胶囊体世界骨架线段逆旋转平移至 Box 的局部坐标系中。在局部空间内，Box 变为中心在原点的标准轴对齐包围盒 $[-hx, hx] \times [-hy, hy]$。
    2.  **6 候选点极值定理**：根据凸集距离极值定理，线段到矩形的最近点必然发生在以下 6 个特征点之一：
        - 胶囊体骨架线段的端点 $A_{\text{loc}}$ 与 $B_{\text{loc}}$（共 2 个）
        - 矩形的 4 个角点 $(\pm hx, \pm hy)$ 在骨架线段上的正交投影点（共 4 个）
    3.  **AABB 夹紧求距**：对这 6 个点分别做 Clamp 操作求出其在矩形表面的最近点，筛选出全局距离最小的点对 $(P_{\text{seg}}, P_{\text{box}})$。
    4.  **分流求解（浅接触与深穿透）**：
        - **浅接触**（点在 Box 外）：法线沿 $P_{\text{box}} - P_{\text{seg}}$，穿透量为 $r_{\text{cap}} - \text{dist}$。
        - **深穿透**（骨架已完全没入 Box 内部）：退化为类似 SAT 的轴向投影测试，找出距离矩形上下左右 4 个面重叠量最小的方向作为弹出法线，防止发生物理发散。
    5.  **坐标回正**：将局部法线与接触点乘上 Box 的世界旋转矩阵还原回世界空间。

---

### 3. 开发复盘：Day 02 攻克的几何陷阱

#### **问题 A：平行线段导致的分母除零发散**
- **现象**：当两个胶囊体水平平行摆放并贴合时，引擎瞬间崩溃或输出 `NaN`。
- **根因**：两线段平行时方向向量点乘的行列式 $\text{denom} = \|\mathbf{d}_1\|^2\|\mathbf{d}_2\|^2 - (\mathbf{d}_1 \cdot \mathbf{d}_2)^2 = 0$，直接相除触发浮点异常。
- **解决**：在 `ClosestPointsBetweenSegments` 中加入行列式零值断言，退化为端点投影法处理平行工况。

#### **问题 B：同心重叠时的法线退化**
- **现象**：当小球中心恰好落在胶囊体骨架线上时，距离向量 $d = (0, 0)$，导致归一化法线变成 $(0, 0)$，两物体死锁穿透。
- **解决**：加入 $\epsilon = 10^{-6}$ 边界判定。当中心完全重叠时，强制使用胶囊体骨架的正交法线作为默认排斥力方向。

---

### 4. 如何验证
运行 `tests/CapsuleTests.cpp`。当前已全绿通过以下几何与物理单元测试：
- ✅ **几何面积与惯量校验**：解析面积输出 `11.141592`，动态密度调整与刚体质量同步无误。
- ✅ **线段最近点投影**：验证了端点内部投影、端点外夹紧等极端边界用例。
- ✅ **Capsule vs Circle 对撞**：单侧接触无水平侧滑，反弹冲量符合动量守恒。
- ✅ **Capsule vs Capsule 十字交叉碰撞**：两线段空间正交逼近，精准生成单接触点流形与分离法线。

**Day 02 运行快照：**
```text
[INFO] >>> Starting V3 002: Capsule Test...
[INFO] area = 11.141592 (expected 11.141592) PASS
[INFO] Mass & UpdateMassData: PASS
[INFO] ClosestPointOnSegment: PASS
[INFO] CapsuleVsCircle: bounced=1 maxVy=7.023336 minY=1.441344 dx=0.000000 -> PASS
[INFO] CapsuleVsCapsule: contactCount=1 normal=(0.000000, 1.000000) contactY=0.985463 restY=3.970926 -> PASS
[INFO] >>> ALL TESTS PASSED <<<
```
---

## 💻 编译与运行
- **开发环境**：Visual Studio 2019 / 2022 (ISO C++11 Standard)
- **编译依赖**：无第三方依赖（纯手搓核心数学与几何算法）