# MyPhysicsEngine2D - V4

一个基于 C++11 构建的高仿真 2D 刚体物理引擎。在 V1 稳健离散动力学、V2 工业级性能架构（动态 AABB 树、休眠岛屿、CCD 雏形）与 V3 基础特性体系（材质、触发器、事件总线、多边形裁剪、基础约束）的基础上，**V4 致力于“全约束家族补全”、“空间级交互查询体系”与“Bullet 持续碰撞全链路打通”**。

## 📌 项目愿景
V4 阶段的目标是将物理引擎打磨至对齐商业级工业物理套件（如 Box2D）的完整度。通过将约束维度从 1~2 自由度推向高阶空间耦合、引入不等式与有界冲击力学，并打通世界级空间索引查询，支撑起包含复杂齿轮传动、滑轮吊臂、硬悬挂战车以及实时鼠标拖拽交互的综合物理沙盒。
- **全约束家族对齐**：完整补齐 Weld、Prismatic、Friction、Rope、Pulley、Gear、Wheel、Mouse 8 类高阶关节，实现机械传动与柔性绳索的工业级模拟。
- **有界与不等式约束解算**：攻克绳关节单侧拉力截断（$\lambda \ge 0$）与摩擦/鼠标关节有界冲量饱和（$\lambda \in [-\mu P, +\mu P]$），掌握拉格朗日乘子约束投影（Constraint Projection）核心算法。
- **世界级多维度空间查询**：基于底层动态 AABB 树封装 `RayCast`（射线检测穿透与最近阻挡）、`QueryAABB`（区域范围重叠）与 `TestPoint`（点选拾取）三套 O(log N) 查询总线。
- **Bullet 级 CCD 深度贯通**：将扫掠包围盒（Swept AABB）与 TOI 时间回溯无缝并入主仿真流程，杜绝高速子弹穿墙，同时维持普通刚体零性能损耗。

---

## 🛠 项目结构 (V4 更新)
```text
MyPhysicsEngine2D/
├── include/
│   └── physics/
│       ├── Collision/
│       │   ├── Shapes/            # Box, Circle, Capsule, Polygon
│       │   ├── DynamicTree.h      # 动态 AABB 树空间索引
│       │   ├── RayCastHit.h       # <--- [V4] 射线检测命中结果封装
│       │   └── ...
│       ├── Dynamics/
│       │   ├── Body.h
│       │   ├── World.h            # <--- [V4] 接入世界查询 API 与 CCD 全流程
│       │   ├── Island.h
│       │   ├── Material.h
│       │   ├── Joint.h            # 关节抽象基类三部曲
│       │   ├── DistanceJoint.h
│       │   ├── SpringJoint.h
│       │   ├── RevoluteJoint.h
│       │   ├── WeldJoint.h        # <--- [V4] 焊接关节 (3 自由度全锁定)
│       │   ├── PrismaticJoint.h   # <--- [V4] 滑块关节 (限位+马达)
│       │   ├── FrictionJoint.h    # <--- [V4] 摩擦关节 (有界冲量饱和)
│       │   ├── RopeJoint.h        # <--- [V4] 绳关节 (不等式单向约束)
│       │   ├── PulleyJoint.h      # <--- [V4] 滑轮关节 (传递比滑轮组)
│       │   ├── GearJoint.h        # <--- [V4] 齿轮关节 (关节间坐标耦合)
│       │   ├── WheelJoint.h       # <--- [V4] 车轮关节 (刚性悬挂+驱动马达)
│       │   └── MouseJoint.h       # <--- [V4] 鼠标关节 (软约束交互拖拽)
│       └── Events/
│           ├── ContactListener.h  # 碰撞/触发器事件监听
│           └── QueryCallback.h    # <--- [V4] 区域与射线查询回调基类
├── src/
│   ├── Dynamics/
│   │   ├── WeldJoint.cpp          # <--- [V4] 3x3 矩阵克莱姆法则求解
│   │   └── ...
│   └── ...
└── README.md
```

---

## 📅 进度跟踪 (V4 14天挑战)

### 第 1 阶段：锁定与平移类关节
- [x] **Day 01: 焊接关节 (WeldJoint) 与 3 自由度全锁定矩阵**
  - 推导 3-DOF 约束方程，组装 $3 \times 3$ 耦合有效质量矩阵 $K$（线-角耦合）。
  - 实现纯 CPU 寄存器级克莱姆法则（Cramer's Rule）手搓求逆，无矩阵库依赖。
  - 实现 3D Baumgarte 速度偏置注入与 3D 冲量热启动（Warm Starting）。
  - 实现基于非线性高斯-赛德尔（NGS）的位置级姿态投影微调，彻底消除下垂变形。
- [ ] **Day 02: 滑块关节骨架 (PrismaticJoint & Limits)**
- [ ] **Day 03: 滑块马达与摩擦关节 (Prismatic Motor & FrictionJoint)**
- [ ] **Day 04: 绳关节 (RopeJoint 不等式单向约束)**

### 第 2 阶段：复合传动与交互关节
- [ ] **Day 05: 滑轮关节 (PulleyJoint 机械倍率与功守恒)**
- [ ] **Day 06: 齿轮关节 (GearJoint 关节耦合器)**
- [ ] **Day 07: 车轮关节 (WheelJoint 硬悬挂与底盘驱动)**
- [ ] **Day 08: 鼠标关节 (MouseJoint 软交互与冲量限幅)**

### 第 3 阶段：世界查询与连续碰撞
- [ ] **Day 09: 世界级查询 API (RayCast / QueryAABB / PointTest)**
- [ ] **Day 10: Bullet 连续碰撞管线全贯通 (Swept AABB + TOI Sub-stepping)**
- [ ] **Day 11: 高速穿墙极端压力测试与弹道验证**

### 第 4 阶段：复合场景与工程收尾
- [ ] **Day 12: 复杂工况展示 1：电梯与起重机 (Elevator & Crane Demos)**
- [ ] **Day 13: 复杂工况展示 2：重装战车与破坏链锤 (Tank & Flail Demos)**
- [ ] **Day 14: 全量回归测试、文档完善与 GitHub 发布**

---

## 🚀 Day 01 进展：焊接关节 (WeldJoint) 与 3 自由度全锁定矩阵

### 1. 技术核心与力学建模：3-DOF 刚性熔接体系
在 V3 中，我们实现的关节最高为 2 自由度（销钉锁死平移，保留旋转）。焊接关节（WeldJoint）则是将两个刚体在拓扑上**彻底电焊熔接为一体**，要求两刚体**锚点世界坐标时刻重合，且相对旋转角度恒定不变**：

#### A. 3 自由度几何与速度约束方程
设初始装配时两刚体相对夹角为 $\theta_{\text{ref}} = \theta_{B0} - \theta_{A0}$：
* **几何位置约束方程（3 维向量）**：
  $$\mathbf{C}(x) = \begin{bmatrix} p_B - p_A \\ \theta_B - \theta_A - \theta_{\text{ref}} \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix} \in \mathbb{R}^3$$
* **速度级导数方程**：
  $$\dot{\mathbf{C}} = \begin{bmatrix} (\mathbf{v}_B + \boldsymbol{\omega}_B \times \mathbf{r}_B) - (\mathbf{v}_A + \boldsymbol{\omega}_A \times \mathbf{r}_A) \\ \omega_B - \omega_A \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}$$
* **$3 \times 6$ 雅可比矩阵 $J$**：
  $$J = \begin{bmatrix} 
  -1 & 0 & r_{Ay} & 1 & 0 & -r_{By} \\ 
  0 & -1 & -r_{Ax} & 0 & 1 & r_{Bx} \\
  0 & 0 & -1 & 0 & 0 & 1
  \end{bmatrix}$$

#### B. $3 \times 3$ 耦合有效质量矩阵 $K$ (Coupled Mass Matrix)
平移与旋转在刚体转动惯量下产生强耦合，约束质量矩阵退化为 $3 \times 3$ 对称正定矩阵 $K = J M^{-1} J^T$：
$$K = \begin{bmatrix} 
K_{11} & K_{12} & K_{13} \\ 
K_{12} & K_{22} & K_{23} \\ 
K_{13} & K_{23} & K_{33} 
\end{bmatrix}$$
* **左上角 $2 \times 2$ 分块**：线位移阻抗（与 RevoluteJoint 完全同构）：
  $$K_{11} = m_A^{-1} + m_B^{-1} + I_A^{-1} r_{Ay}^2 + I_B^{-1} r_{By}^2$$
  $$K_{22} = m_A^{-1} + m_B^{-1} + I_A^{-1} r_{Ax}^2 + I_B^{-1} r_{Bx}^2$$
  $$K_{12} = -I_A^{-1} r_{Ax} r_{Ay} - I_B^{-1} r_{Bx} r_{By}$$
* **右下角元素 $K_{33}$**：纯角位移阻抗：
  $$K_{33} = I_A^{-1} + I_B^{-1}$$
* **线角交叉耦合项 $K_{13}, K_{23}$**（平移拉扯直接引起转动的耦合核心）：
  $$K_{13} = -I_A^{-1} r_{Ay} - I_B^{-1} r_{By}, \qquad K_{23} = I_A^{-1} r_{Ax} + I_B^{-1} r_{Bx}$$

---

### 2. 核心算法设计与执行流水线

```text
[initVelocityConstraints] ──► 计算 rA, rB ──► 组装 3x3 矩阵 K ──► 注入 3D 偏置 m_bias ──► 热启动 (线冲量 + 角扭矩)
                                                                                                    │
                                                                                                    ▼
[solveVelocityConstraints] ──► C_dot 组装 ──► 克莱姆法则手搓解 K·λ = B ──► 双向累加冲量 ──► 原子回写速度与角速度
                                                                                                    │
                                                                                                    ▼
[solvePositionConstraints] ──► 实时重算当前位姿 ──► 非线性位置投影 (NGS) ──► 直接微调 Position/Rotation ──► 刷新 AABB
```

1. **手搓克莱姆法则（Cramer's Rule）解析求逆**：
   抛弃外部矩阵库开销，直接通过 3 个列向量的标量三重积 $\det = \mathbf{c}_1 \cdot (\mathbf{c}_2 \times \mathbf{c}_3)$ 展开伴随矩阵求解，耗时降至十几纳秒。
2. **三维双向无界冲量更新**：
   平移和旋转均为双向等式约束（允许正负推拉扭转），冲量 $\boldsymbol{\lambda} \in \mathbb{R}^3$ 无需做任何非负截断（无 Clamp）。
3. **高精度非线性位置投影 (NGS)**：
   在位置解算中，将 3D 位置冲量拆解为线位移量与力矩臂叉乘 $(\mathbf{r} \times \mathbf{P}_{\text{linear}} + \tau)$，直接回写几何位置与角度并刷新空间索引，消除视觉下垂。

---

### 3. 开发复盘：Day 01 攻克的隐秘陷阱

#### **问题 A：角冲量热启动遗漏导致的“低频点头微颤”**
- **现象**：悬臂梁在重力作用下静止挂载时，每一帧刚体都会先微微下垂再被拉回，呈现微颤。
- **根因**：在 `initVelocityConstraints` 中提取了 `torqueImpulse = m_impulse.z;`，但仅调用了 `applyImpulse(P, r)`，遗漏了将纯旋转锁定冲量施加回刚体角速度。
- **解决**：补齐纯角度锁定的热启动扭矩：$\omega_A \mathrel{-}= I_A^{-1} \tau$、$\omega_B \mathrel{+}= I_B^{-1} \tau$，初始速度误差瞬间归零。

#### **问题 B：局部变量污染导致的“偏置读脏数据”**
- **现象**：刚体在焊接处产生不可预测的巨大虚假拉力。
- **根因**：写出了 `m_bias.x = ...; Vector3 bias = { ... };`，定义了局部变量但成员变量 `m_bias.y` 和 `m_bias.z` 完全未初始化，迭代求解器读到了内存随机垃圾值。
- **解决**：直接对成员变量 `m_bias` 进行完整 3 维赋值并加入 `dt > 0` 防除零保护。

#### **问题 C：位置解算中的“量纲与力矩臂时序倒挂”**
- **现象**：开启位置解算后刚体发生高速旋转炸飞。
- **根因**：
  1. 误用了刚体角速度 `getAngularVelocity()` 计算几何误差；
  2. 修正坐标时漏乘了质量倒数 $m^{-1}$；
  3. 角度修正漏掉了线冲量力矩臂叉乘项 $(\mathbf{r} \times \mathbf{P})$。
- **解决**：严格重写为非线性位置投影公式，使用 `getRotation()`，乘满质量与转动惯量标量，彻底稳固。

---

### 4. 单元测试验证与物理表现 (Verification)

运行 `tests/WeldJointTests.cpp`。当前已全绿通过以下核心全刚性测试：

#### ① 悬臂梁极限抗弯曲测试 (`CantileverBeamRigidity`)
* **验证场景**：一端固定在静态墙体 $(0, 0)$，水平焊接一根长度 4.0 米的长条刚体，末端施加 100N 向下重力与载荷。
* **物理表现**：模拟 300 帧后，连杆始终维持水平，末端垂直下垂位移 $< 10^{-4}$ 米，旋转角漂移严格控制在 $10^{-5}$ 弧度以内，**展现出犹如单一实心物体的绝对刚性**。

#### ② 刚性组合体撞击反弹测试 (`CompositeBodyBehavior`)
* **验证场景**：两个矩形方块通过 WeldJoint 焊接成“L”形异形刚体，从高空砸向斜坡。
* **物理表现**：触地反弹全过程中，两刚体角速度严格同步，焊接缝隙零脱节、零相对扭动，如同单个凹多边形整体平稳反弹。

#### ③ 自身碰撞过滤验证 (`CollideConnectedFilter`)
* **验证场景**：两刚体在焊接处产生大面积几何重叠。
* **物理表现**：设置 `collideConnected = false` 后，窄相求解器完全屏蔽了焊接部位的推挤冲量，**零自激震荡、零数值爆炸**。

**Day 01 运行快照：**
```text
[INFO] >>> Starting V4 001: WeldJoint Test...
[INFO] CantileverBeam: length=4.000000 tipDrift=0.000000 angleDrift=0.000000 -> PASS
[INFO] CompositeBody: relativeAngle=0.000000 seamSeparation=0.000000 -> PASS
[INFO] CollideConnectedFilter: overlapping=1 noExplosion=1 -> PASS
[INFO] >>> ALL TESTS PASSED <<<
```

---

## 💻 编译与运行
- **开发环境**：Visual Studio 2019 / 2022 (ISO C++11 Standard)
- **编译依赖**：无第三方依赖（纯手搓核心数学与几何算法）