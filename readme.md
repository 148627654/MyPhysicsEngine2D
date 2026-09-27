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
- [x] **Day 03: 通用凸多边形表示与惯性属性 (Convex Polygon)**
- [x] **Day 04: 多边形接触流形裁剪 (Sutherland-Hodgman Clipping)**

### 第 2 阶段：生命周期与事件系统
- [x] **Day 05: 接触状态缓存与生命周期判定 (Contact Cache)**
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
这里是为你量身编写的 **V3 - Day 03 任务总结（README 增补内容）**。

你可以直接将其复制并追加到你的 `README.md` 中，同时将进度表中的 `Day 03` 勾选为 `[x]`：

---

## 🚀 Day 03 进展：通用凸多边形 (Convex Polygon) 与格林公式惯量解析积分

### 1. 技术核心：几何严谨性与质心归零法则
通用凸多边形是刚体物理引擎表达任意复杂刚体的基石。不同于游戏引擎纯渲染用的网格，物理引擎对几何多边形有着极其苛刻的**拓扑和动力学约束**：

#### A. 凸性闭环与逆时针 (CCW) 拓扑约束
为了确保分离轴定理（SAT）和后续的接触裁剪有效，输入点集必须满足：
1. **顶点数受限**：设定 `MAX_VERTICES = 8`（工业界标准如 Box2D），在绝大多数场景表达能力与 CPU 缓存行命中率之间取得极致平衡。
2. **闭环拐角正定性**：遍历所有 $N$ 个闭环拐角，根据二维叉积严格断言前进方向内部始终在左侧：
   $$(P_{i+1} - P_i) \times (P_{i+2} - P_{i+1}) > 0$$
   *彻底拦截顺时针顶点序（CW）、共线退化边以及内凹角（Concave）。*

#### B. 动力学铁律：质心归零化 (Centroid Centering)
在刚体动力学中，如果形状的局部原点 $(0,0)$ 不在物理质心上，旋转积分时会导致刚体绕“偏心点”旋转，从而产生荒谬的自旋伪力矩。
* **双阶段平移机制**：
  1. 先用原始坐标计算几何形心 $C$。
  2. 将所有局部顶点强制统一平移：$P'_i = P_i - C$。
  3. 使多边形在局部坐标系下严格满足 $\iint \mathbf{r} \, dA = \mathbf{0}$。
  *(测试实测：偏心三角形在 `Set()` 后，重算质心精确达到 `(-0.000000, -0.000000)`)*

---

### 2. 数学积分与核心算法深度详解

#### ① 格林公式与三角形拆分积分（面积、转动惯量）
对于质心归零后的凸多边形，我们以局部原点 $(0,0)$ 为中心建立微元三角形扇面，使用**格林公式（Green's Theorem）**将二重积分转化为边缘曲线闭环积分：
* **鞋带公式（Shoelace Formula）求面积**：
  $$Area = \frac{1}{2} \sum_{i=0}^{n-1} (P_i \times P_{i+1})$$
* **二次惯性矩解析闭式解 (Closed-form Polar Inertia)**：
  对三角形微元 $\iint (x^2 + y^2) dx dy$ 解析求导，导出无需二重采样的精确积分式：
  $$D = P_i \times P_{i+1}$$
  $$I_{\Delta} = \frac{D}{12} \left( \|P_i\|^2 + P_i \cdot P_{i+1} + \|P_{i+1}\|^2 \right)$$
  $$I_{\text{total}} = \rho \sum_{i=0}^{n-1} I_{\Delta_i}, \quad m = \rho \cdot Area$$

* **Box vs Polygon 理论对照大杀器**：
  长宽为 $(2, 4)$、密度为 $1.5$ 的矩形：
  - 矩形理论值：$Area = 8.0, \quad m = 12.0, \quad I = \frac{1}{12} \times 12 \times (2^2 + 4^2) = 20.0$
  - `Polygon` 三角微元积分输出：`area=8.000000, mass=12.000000, inertia=20.000000`
  *两套完全不同的数学体系在浮点数 $10^{-6}$ 精度下达成完美收敛吻合！*

---

#### ② Cyrus-Beck 局部空间半空间射线裁剪算法
```cpp
bool Polygon::RayCast(RayCastOutput* output, RayCastInput& input, const Vector2& position, float rotation);
```
* **算法思想**：凸多边形是 $N$ 个半空间（Half-spaces）的交集。
  1. **坐标系逆变换**：将世界射线通过刚体位姿变换到局部空间，直接复用预计算的局部单位外法线 `m_normals`。
  2. **区间收窄 (Interval Clipping)**：
     - 入边 ($d \cdot \mathbf{n}_i < 0$)：持续推后进入时间 $t_{\text{lower}} = \max(t_{\text{lower}}, t)$，记录命中边索引。
     - 出边 ($d \cdot \mathbf{n}_i > 0$)：持续提前离开时间 $t_{\text{upper}} = \min(t_{\text{upper}}, t)$。
     - 若 $t_{\text{lower}} > t_{\text{upper}}$，说明射线从多边形外侧擦过，立即剪枝返回 `false`。
  3. **法线正交性校验**：
     测试中正五边形在参数 $t = 0.329870$ 处被命中，命中外法线与被击中线段的点积严格为 `0.000000`，证明几何法线绝对垂直于碰撞边界。

---

### 3. 开发复盘：Day 03 攻克的几何与工程陷阱

#### **问题 A：指针数组与模长语义混淆（崩溃隐患）**
- **现象**：`Polygon::Set` 遍历时突发内存非法访问崩溃（Access Violation）。
- **根因**：写出了 `int length = vertices->Length();`。`vertices` 是原始指针，此写法将第 0 个矢量的模长 $\sqrt{x^2+y^2}$ 强转为循环上限，丢弃了传参的真实顶点数 `count`。
- **解决**：改用传入的 `count` 变量，并加上严格边界断言 `3 <= count <= MAX_VERTICES`。

#### **问题 B：多边形首尾拐角的“闭环漏检”**
- **现象**：一个最后一个角凹陷的“飞镖形”多边形被非法判定为合法凸多边形。
- **根因**：循环仅检查了下标 $1 \dots (n-1)$ 的角，遗漏了由第 $N-1$ 点连向第 $0$ 点、再由第 $0$ 点连向第 $1$ 点构成的闭合拐角。
- **解决**：采用环形取模索引 `(i + 1) % count` 和 `(i + 2) % count`，完整覆盖多边形的全部 $N$ 个内角。

---

### 4. 如何验证
运行 `tests/PolygonTests.cpp`。当前已全绿通过以下几何、动力学与射线测试：
- ✅ **矩形等价性对照**：四角点构造的 Polygon 与 Box 在面积、质量、转动惯量上达成 $0$ 误差一致。
- ✅ **质心平移修正**：偏心三角形经过归零化后质心精准归零至 $(0, 0)$，面积严格守恒为 $6.0$。
- ✅ **非法几何防御拦截**：顺时针顶点、凹多边形、点数不足 3 均被 `Set()` 精准拒绝。
- ✅ **Cyrus-Beck 射线检测**：正五边形求交命中比例精确匹配理论值 $0.329870$，且法线正交积为 $0$。

**Day 03 运行快照：**
```text
[INFO] >>> Starting V3 003: Polygon Test...
[INFO] Box vs Polygon: area=8.000000 mass=12.000000 inertia=20.000000 (box inertia=20.000000) -> PASS
[INFO] CentroidShift: centroid=(-0.000000, -0.000000) -> PASS
[INFO] Validation: rejectCW=1 rejectConcave=1 rejectFewVerts=1 -> PASS
[INFO] RayCast: hit=1 t=0.329870 normal=(-0.951057, -0.309017) dot(n,edge)=0.000000 -> PASS
[INFO] >>> ALL TESTS PASSED <<<
```
---
这里是为你量身编写的 **V3 - Day 04 任务总结（README 增补内容）**。

你可以直接将其复制并追加到你的 `README.md` 中，同时将进度表中的 `Day 04` 勾选为 `[x]`：

---

## 🚀 Day 04 进展：Sutherland-Hodgman 接触流形裁剪与多边形碰撞解算

### 1. 技术核心与物理突破：双点接触线段化
在物理引擎中，如果碰撞只能生成 1 个点，平躺的刚体会因为缺乏力矩支撑而发生高频振荡或倾覆。今天我们攻克了多边形碰撞最具含金量的核心——**基于 SAT 分离轴判定与 Sutherland-Hodgman 算法的高精度接触流形裁剪**：

#### A. 双向 SAT 仲裁与参考面提取
* **双重否决机制**：调用 `FindMaxSeparation` 进行双向测试（A 测 B 与 B 测 A），任意轴存在分离（$\text{sep} > 0$）即剪枝退出。
* **参考面 (Reference) 仲裁**：比较两者的穿透深度，选取穿透更浅的轴所在多边形作为参考面，另一方作为附着面。引入 `flip` 标志位确保法向始终严格由 BodyA 指向 BodyB。

#### B. Sutherland-Hodgman 三道半平面裁剪流水线
附着边线段进入裁剪流水线后，经受参考边构筑的三个半空间层层截断：
1. **左侧切面**：截断超出参考边起点的悬空线段。
2. **右侧切面**：截断超出参考边终点的悬空线段。
3. **正面深度测量**：计算剩余端点沿参考面法向的有向深度，舍弃未侵入点（$\text{depth} \le 0$），保留有效接触点（至多 2 个）。

```text
       ┌───────────────┐
       │ Incident Poly │
       └───[ I1─────I2 ]───  <--- 附着边 (Incident Edge)
             │       │
      ←──────[ R1───R2 ]──────→  <--- 参考边 (Reference Edge，构筑左右侧面与正面)
       ┌─────────────────┐
       │ Reference Poly  │
       └─────────────────┘
```

---

### 2. 核心算法验证与物理表现

#### ① 面面接触与双点力偶支撑 (`FaceFace`)
* **工况**：两个宽 2 高 4 的矩形多边形垂直上下叠放，穿透量 $0.05$。
* **实测输出**：`hit=1 count=2 c0=(-1.000000, 0.950000) c1=(1.000000, 0.950000) n=(0.000000, 1.000000)`。
* **物理意义**：算法在接触面上精确截出了长为 $2.0$ 的水平接触线段，法线垂直向上。求解器在两端分别施加冲量，形成抵抗倾覆的力偶，**多边形自此能够稳如磐石地平躺堆叠**。

#### ② 角面接触自然退化 (`CornerFace`)
* **工况**：上方多边形倾斜 $45^\circ$（尖角朝下）撞击水平多边形。
* **实测输出**：`hit=1 count=1 c0=(-0.000000, 0.950000)`。
* **物理意义**：算法自动在第三道正面裁剪中舍弃了悬空的顶点，自然退化为单点支撑，接触点精准锁定在尖角顶点。

#### ③ 错位重叠裁剪 (`ClippedOverlap`)
* **工况**：上方多边形向右错位平移半个身位后压在下方多边形上。
* **实测输出**：`count=2 c0=(0.000000, 0.950000) c1=(1.000000, 0.950000)`。
* **物理意义**：左侧悬空端点被第一道侧平面精准剪断，生成的 2 个接触点严格收敛在 $[0, 1]$ 的真实几何重叠区间内，杜绝了“空中假着力点”。

#### ④ 多边形与圆碰撞 (`PolygonVsCircle`)
* **工况**：小球落在 $45^\circ$ 斜面上。
* **实测输出**：`contact=1 n=(-0.707107, 0.707107) dot(n,face)=0.000000 minVx=-2.858334`。
* **物理意义**：法线正交积严格等于 $0.0$，且在冲量解算下小球获得了 $-2.86\,\text{m/s}$ 的横向反弹初速度，混合碰撞动力学验证通过。

---

### 3. 开发复盘：Day 04 攻克的工程死穴

#### **问题 A：嵌套结构体的“自上而下”声明依赖**
- **现象**：`FindIncidentEdge` 编译报错 `“ClipVertex”: 未声明的标识符`。
- **根因**：C++ 类内部函数参数所依赖的嵌套类型必须在其上方完成定义。
- **解决**：调整声明顺序，将 `struct ClipVertex` 置于所有成员函数声明之前。

#### **问题 B：SAT 测量中的“绝对值”逻辑陷阱**
- **现象**：圆心深穿透进多边形内部时，系统误判为“圆在外部未碰撞”。
- **根因**：计算圆心到边的距离时误加了 `std::abs`，导致背面原本深深在内侧的负有向距离（如 $-10.0$）被反转为 $+10.0$，触发了分离轴剪枝。
- **解决**：彻底移除绝对值运算，使用纯正的带符号有向距离进行 SAT 极值筛选。

---

### 4. 如何验证
运行 `tests/PolygonCollisionTests.cpp`。当前已全绿通过以下核心流形与几何测试：
- ✅ **面面平躺测试**：精准生成 2 接触点，法线水平垂直无偏移。
- ✅ **角面点接触测试**：自适应退化为 1 接触点。
- ✅ **错位裁剪测试**：悬空端点被双侧平面精准拦截截断。
- ✅ **多边形与圆混合对撞**：Voronoi 面/角区域判定生效，斜面反弹法向严格守恒。

**Day 04 运行快照：**
```text
[INFO] >>> Starting V3 004: Polygon Collision Test...
[INFO] FaceFace: hit=1 count=2 c0=(-1.000000,0.950000) c1=(1.000000,0.950000) n=(0.000000,1.000000) -> PASS
[INFO] CornerFace: hit=1 count=1 c0=(-0.000000,0.950000) -> PASS
[INFO] ClippedOverlap: count=2 c0=(0.000000,0.950000) c1=(1.000000,0.950000) -> PASS
[INFO] PolygonVsCircle: contact=1 n=(-0.707107,0.707107) dot(n,face)=0.000000 minVx=-2.858334 -> PASS
[INFO] >>> ALL TESTS PASSED <<<
```
这里是为你量身编写的 **V3 - Day 05 任务总结（README 增补内容）**。

你可以直接将其复制并追加到你的 `README.md` 中，同时将进度表中的 `Day 05` 勾选为 `[x]`：

---

## 🚀 Day 05 进展：接触状态缓存器 (Contact Cache) 与 时序生命周期状态机

### 1. 技术核心：为物理世界注入“时序记忆”
在传统的离散物理模拟中，碰撞检测是“用完即扔”的无状态计算。但对于上层的游戏逻辑与事件系统而言，业务强依赖于状态变迁（如击中音效需在刚碰上时播放一次、持续燃烧伤害需在接触期间每秒结算、离开地面才触发起跳状态）。

今天我们实现了完整的 **接触状态缓存器 (`ContactManager`)**，将物理世界从“瞬时判定”升维为“具备跨帧时序记忆的状态机”：

#### A. 无序碰撞对 64 位整型压缩 (`ContactKey`)
对于任意两个刚体 $A$ 与 $B$，无论遍历顺序如何，它们之间的物理接触在逻辑上是**绝对等价且唯一**的：
* 提取两刚体全局唯一自增标识 `idA` 与 `idB`，按大小排序消除次序性：
  $$\text{id}_1 = \min(\text{idA}, \text{idB}), \quad \text{id}_2 = \max(\text{idA}, \text{idB})$$
  $$\text{Key} = (\text{uint64\_t}(\text{id}_1) \ll 32) \mid \text{uint64\_t}(\text{id}_2)$$
* 实现了哈希表内部 $O(1)$ 复杂度的查找与天然碰撞去重。

#### B. 双缓冲集合差集算法 (Set-Difference State Machine)
维护上一帧接触集合 $S_{\text{prev}}$ 与当前帧接触集合 $S_{\text{curr}}$，通过三向集合差集，自动且无遗漏地推导出三态生命周期：
* **`Enter`（初次接触）**：$S_{\text{curr}} \setminus S_{\text{prev}}$（本帧新出现，上帧未记录）。
* **`Stay`（持续保持）**：$S_{\text{curr}} \cap S_{\text{prev}}$（本帧与上帧连续重叠）。
* **`Exit`（脱离接触）**：$S_{\text{prev}} \setminus S_{\text{curr}}$（上帧尚存，本帧检测已分离）。发生脱离时完整保留上一帧的最后流形记忆，使得外部能够准确读取脱离瞬态。

---

### 2. 核心架构与四大安全管线实现

在 `ContactManager` 内部确立了清晰的四阶管线：
1. **`AddContact`（窄相实时归档）**：窄相检测成功后即刻打包刚体指针、几何流形与 `isTrigger` 标识写入 $S_{\text{curr}}$。
2. **`UpdateStates`（生命周期集中裁决）**：在冲量解算前统一跑完双向差集对比，将所有裁定好的记录扁平化写入 `m_lifecycleRecords`。
3. **`EndFrame`（零拷贝轮转）**：通过 C++11 `std::move` 实现哈希表底层桶指针的 $O(1)$ 零拷贝所有权交接，杜绝深拷贝消耗。
4. **`OnBodyDestroyed`（野指针拆弹安全门）**：当刚体被外界销毁时，以安全迭代器擦除该 Body 关联的所有当前、历史与待派发记录，彻底根除 0xC0000005 悬挂指针崩溃风险。

---

### 3. 开发复盘：Day 05 攻克的暗坑

#### **问题 A：布尔守卫条件的“德摩根定律陷阱”**
- **现象**：`AddContact` 的指针防御失效，传入空指针时没有提前返回，直接崩溃。
- **根因**：写出了 `if (!bodyA && !bodyB && bodyA != bodyB) return;`。只有两者同时为空且自己不等于自己时才触发，导致条件恒为假。
- **解决**：改写为标准的析取守卫语句 `if (!bodyA || !bodyB || bodyA == bodyB) return;`。

#### **问题 B：范围 for 循环中的迭代器失效（Crash）**
- **现象**：在 `OnBodyDestroyed` 中遍历哈希表并调用 `erase(key)` 时，程序在下一轮迭代突发野指针异常。
- **根因**：C++ 范围 for 循环在末尾隐式执行 `++it`，删除当前节点后已导致迭代器悬挂失效。
- **解决**：重构为经典显式迭代器循环，在删除分支使用 `it = map.erase(it)` 接住下一个有效迭代器指针。

#### **问题 C：类型默认构造函数被编译器隐式删除 (E1790)**
- **现象**：定义 `ContactRecord record;` 触发编译错误，提示默认构造函数是“已删除的函数”。
- **根因**：成员变量 `Manifold` 编写了自定义构造函数，导致编译器自动注销了无参默认构造。
- **解决**：在 `Manifold.h` 中显式补充 `Manifold() = default;`。

---

### 4. 测试验证与数据剖析 (Verification)

运行 `tests/ContactCacheTests.cpp`。当前已全绿通过以下核心状态机单元测试：

#### ① 完整生命周期流转 (`Lifecycle Enter->Stay->Exit`)
* **验证场景**：小球自然下落撞击地面，随后被代码手动移走。
* **状态记录**：第 1 帧精准捕获 `Enter`；随后连续多帧稳定输出 `Stay`；分离瞬间无缝捕获 `Exit`；下一帧彻底清空。三态变迁无任何重叠或漏报。

#### ② 触发器时序透传 (`TriggerEvent`)
* **实测输出**：`enter=1 stay=13 exit=1 allTrigger=1 minVy=-19.599997`。
* **物理意义**：刚体穿过 Trigger 区域，在 13 帧的穿透时间内持续产生 `Stay` 状态，离开时产生 `Exit`。且因为求解器旁路生效，小球垂直速度在重力下平滑加速到 $-19.6\,\text{m/s}$，动能零损耗。

#### ③ 多对独立状态隔离性 (`Isolation`)
* **实测输出**：`enterAB=1 stayAB=59 enterBC=1 exitBC=1 A_y=1.470929`。
* **物理意义**：A-B 两物体在长达 59 帧的静止堆叠中持续保持 `Stay`。中途 C 物体切入碰撞 B（触发 `enterBC`）随后离开（触发 `exitBC`），A-B 的状态机完全不受邻居干扰，物体 A 始终稳如磐石停在 $Y=1.47$。

**Day 05 运行快照：**
```text
[INFO] >>> Starting V3 005: Contact Event Test...
[INFO] Lifecycle Enter->Stay->Exit: PASS
[INFO] TriggerEvent: enter=1 stay=13 exit=1 allTrigger=1 minVy=-19.599997 -> PASS
[INFO] Isolation: enterAB=1 stayAB=59 enterBC=1 exitBC=1 A_y=1.470929 -> PASS
[INFO] >>> ALL TESTS PASSED <<<
```
这里是为你量身编写的 **V3 - Day 06 任务总结（README 增补内容）**。

你可以直接将其复制并追加到你的 `README.md` 中，同时将顶部进度表中的 `Day 06` 勾选为 `[x]`（标志着第二阶段：生命周期与事件系统圆满收官！）：

---

## 🚀 Day 06 进展：观察者模式与事件派发总线 (Event System)

### 1. 技术核心与架构设计：打通业务交互的“最后一公里”
至 Day 05 为止，引擎底层已具备了高精度的碰撞流形与状态机推导能力，但外部游戏代码依然无法直接感知。今天我们基于**观察者模式（Observer Pattern）**构建了一套解耦的**事件派发总线**，正式宣告引擎从“纯动力学模拟器”蜕变为“具备业务感知能力的游戏级物理引擎”：

#### A. 延迟派发架构与重入防御 (Deferred Event Dispatch)
在物理引擎开发中，最经典的崩溃莫过于用户在回调函数内部执行结构变更（如 `OnCollisionEnter` 中调用 `DestroyBody` 销毁怪物）：
* **同步派发死穴**：如果在求解循环（Island Solver）内部直接调用回调，销毁刚体会导致当前迭代器瞬间悬挂，引发 0xC0000005 内存访问越界。
* **延迟派发流水线**：
  整个 `World::Step` 物理计算全流程（积分 $\rightarrow$ 宽相 $\rightarrow$ CCD $\rightarrow$ 岛屿求解）严禁触发任何外部回调。
  直到所有力学计算**彻底落幕**，才在帧末通过 `DispatchContactEvents()` 集中派发事件。即使上层在回调中随心所欲地 `DestroyBody` 或 `CreateBody`，底层计算管线依然坚如磐石！

#### B. 触发器语义归一化 (Semantic Normalization)
在无序碰撞对 $(A, B)$ 中，触发器可能是 $A$ 也可能是 $B$。为了向游戏开发者提供最直观的 API，我们在封装 `TriggerEvent` 时进行了语义提取：
* 自动判定并将触发器赋予 `triggerBody`，受测者赋予 `otherBody`。
* 开发者无需写繁琐的 `isTrigger` 双向判断，直接取用即可。

---

### 2. 核心架构与事件生命周期管线

```text
[物理计算全闭环] ──► Island Solver 求解结束，冲量与位置全部收敛
                             │
                             ▼
[事件分发总线]   ──► World::DispatchContactEvents()
                     ├─ 提取 ContactManager 本帧生命周期记录
                     ├─ 遍历判定 isTrigger，分流装配 CollisionEvent / TriggerEvent
                     └─ 触发外部注册的 ContactListener 虚接口
                             │
                             ▼
[帧末状态交接]   ──► ContactManager::EndFrame() (双缓冲零拷贝移交)
```

---

### 3. 开发复盘：Day 06 攻克的工程死穴

#### **问题 A：派发时机过早导致的“求解器踩踏崩溃”**
- **现象**：在回调中销毁刚体后，程序在 `BuildAndSolveIslands` 阶段发生空指针异常。
- **根因**：原先事件更新位于第 2.5 步，在此处派发事件导致刚体在岛屿解算前被销毁，解算器读到了已被 `delete` 的刚体。
- **解决**：确立“延迟派发”黄金法则，将 `DispatchContactEvents()` 严格后置在第 5 步（Island Solver 结束后），确保力学流程完全闭环。

#### **问题 B：多态虚基类的默认空实现**
- **设计考量**：在 `ContactListener` 中为 6 个纯虚接口全部赋予默认空函数体 `{}`。
- **收益**：上层业务开发者只需按需重写自己关心的事件（如只重写 `OnTriggerEnter`），无需为了监听单一事件而被迫写满 6 个空函数。

---

### 4. 测试验证与数据剖析 (Verification)

运行 `tests/EventSystemTests.cpp`。当前已全绿通过以下核心事件总线测试：

#### ① 碰撞实体回调捕获 (`BounceCallback`)
* **实测输出**：`enter=1 exit=1 n=(-0.000000, 1.000000) -> PASS`。
* **物理意义**：小球砸向地面并弹起，监听器在碰撞瞬间精确捕获到 1 次 `OnCollisionEnter`，反弹腾空瞬间捕获到 1 次 `OnCollisionExit`。法线精准保持为垂直地面的 `(0, 1)`，无任何多余的虚假事件扰动。

#### ② 实体碰撞与触发器完全隔离 (`TriggerIsolation`)
* **实测输出**：`triggerEnter=1 triggerExit=1 collisionEnter=0 -> PASS`。
* **物理意义**：小球高速穿过金币传感器（Trigger），成功派发 `OnTriggerEnter` 与 `OnTriggerExit`，而实体物理碰撞 `OnCollisionEnter` 触发计数严格为 0，两者事件流泾渭分明，绝不串门。

#### ③ 死斗大杀器：回调内安全自毁 (`SafeDestruction`)
* **实测输出**：`destroyCalls=1 bodies 2->1 -> PASS`。
* **物理意义**：在 `OnCollisionEnter` 回调内部直接执行 `world->DestroyBody(event.bodyB)`！引擎平稳执行至帧末，世界刚体数平滑由 2 减至 1，无任何内存崩溃或野指针泄漏，**高强度验证了延迟派发架构的绝对安全性**！

**Day 06 运行快照：**
```text
[INFO] >>> Starting V3 006: Contact Callback Test...
[INFO] BounceCallback: enter=1 exit=1 n=(-0.000000,1.000000) -> PASS
[INFO] TriggerIsolation: triggerEnter=1 triggerExit=1 collisionEnter=0 -> PASS
[INFO] SafeDestruction: destroyCalls=1 bodies 2->1 -> PASS
[INFO] >>> ALL TESTS PASSED <<<
```
---
## 💻 编译与运行
- **开发环境**：Visual Studio 2019 / 2022 (ISO C++11 Standard)
- **编译依赖**：无第三方依赖（纯手搓核心数学与几何算法）