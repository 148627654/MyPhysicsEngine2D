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
  - 实现物理材质与几何形状完全解耦，定义密度 $\rho$、恢复系数 $e$、动静摩擦力 $\mu_s, \mu_d$。
  - 实现 `CombineMode`（Average、Minimum、Multiply、Maximum）材质混合仲裁策略。
  - 实现 `UpdateMassData` 虚接口与质量/转动惯量动态委托回写机制。
  - 实现解算器对 `isTrigger` 的物理冲量旁路（Solver Bypass），支持穿透同时保留相交数据。
- [ ] **Day 02: 胶囊体碰撞 (Capsule Collider)**
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

---

## 💻 编译与运行
- **开发环境**：Visual Studio 2019 / 2022 (ISO C++11 Standard)
- **编译依赖**：无第三方依赖（纯手搓核心数学与几何算法）