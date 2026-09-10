这份 **V3 业务与特性丰富版（Feature-Complete Edition）** 14天开发进度表，专门针对你提出的 **Joint、Capsule、Polygon、Event System、Material** 进行逻辑编排。

在物理引擎架构中，关节（Joint/Constraint）和形状扩展（Capsule/Polygon）都需要依赖底层的数学求解框架，而事件与材质系统则负责将引擎的能力向“游戏/上层应用”输出。以下是为你制定的执行路线图：

---
# 14天开发进度表

#### 第1阶段：材质属性与复杂几何体扩展（第1-4天）
*   **第1天：材质系统 (Material) 与 触发器 (Trigger)**
    *   定义 `Material` 结构体：包含静态摩擦力 `staticFriction`、动态摩擦力 `dynamicFriction`、弹性恢复系数 `restitution`、密度 `density`。
    *   **材质混合策略**：实现 `CombineMode`（如两物体接触时，弹性取 $\max(e_1, e_2)$，摩擦力取几何平均数 $\sqrt{\mu_1 \cdot \mu_2}$）。
    *   **Trigger 机制**：为 Collider 增加 `isTrigger / isSensor` 标志，碰撞时只派发事件，跳过冲量解算流程。
*   **第2天：胶囊体碰撞 (Capsule Collider)**
    *   数据结构：定义为端点 $A, B$ 与半径 $r$（或中心点、半长、朝向和半径）。
    *   **数学实现**：胶囊体与其他形状的碰撞可等价为“线段到形状的最近距离”。
    *   实现 `CapsuleVsCircle`、`CapsuleVsCapsule`（求两线段间最短线段）。
    *   计算胶囊体的质量与转动惯量（组合法：矩形 + 两半圆）。
*   **第3-4天：通用凸多边形 (Convex Polygon & SAT 升级)**
    *   **第3天**：多边形表示与惯性属性。使用三角剖分法（格林公式）计算任意凸多边形的质心、面积和转动惯量；增加凸性验证（Cross Product 符号一致性）。
    *   **第4天**：多边形碰撞流形裁剪（Sutherland-Hodgman 裁剪算法）。找出参考面（Reference Edge）与附着面（Incident Edge），裁剪生成 **2 个接触点**（形成接触线段），让多边形能够稳定平躺。

#### 第2阶段：生命周期与事件系统（第5-6天）
*   **第5天：接触状态缓存器 (Contact Cache)**
    *   在 `World` 中维护上一帧与当前帧的碰撞对集合（使用物体的唯一 ID 对组合成哈希键）。
    *   通过集合差集，准确推导碰撞生命周期状态：
        *   上帧无，本帧有 $\rightarrow$ **`OnCollisionEnter`** / **`OnTriggerEnter`**
        *   上帧有，本帧有 $\rightarrow$ **`OnCollisionStay`** / **`OnTriggerStay`**
        *   上帧有，本帧无 $\rightarrow$ **`OnCollisionExit`** / **`OnTriggerExit`**
*   **第6天：事件调度与监听器 (Event System)**
    *   实现 `ContactListener` 抽象基类或基于 `std::function` 的回调接口。
    *   封装完整的事件参数类 `CollisionEvent`（携带碰撞双方指针、碰撞法线、接触点冲量大小）。
    *   **解耦设计**：物理迭代步骤中收集事件队列，在 `World::Step` 末尾统一分发，防止用户在回调里修改物理世界导致迭代器失效（迭代安全）。

#### 第3阶段：约束与关节系统（第7-11天，核心硬核算法）
*   **第7天：约束求解器骨架 (Constraint Framework)**
    *   设计 `Joint` 基类，接入解算器循环（`InitVelocityConstraints`, `SolveVelocityConstraints`, `SolvePositionConstraints`）。
    *   引入 **Baumgarte 稳定化方法**（Position Error Bias），防止关节受力时间过长后发生“拉长/脱臼”漂移。
    *   扩展岛屿系统（Island）：关节连接的两个刚体必须属于同一个岛。
*   **第8天：距离关节 (Distance Joint)**
    *   约束方程：$C(x) = \|p_2 - p_1\| - L = 0$（保持物体两锚点距离恒定为 $L$）。
    *   推导速度雅可比矩阵（Jacobian），计算拉格朗日乘子 $\lambda$ 并施加修正冲量。
    *   **验证**：实现不可拉伸的绳索或刚性连杆。
*   **第9天：弹簧阻尼关节 (Spring / Soft Distance Joint)**
    *   在距离关节基础上引入软约束参数：固有频率（Frequency，Hz）与阻尼比（Damping Ratio $\zeta$）。
    *   使用**隐式欧拉法**推导弹簧冲量方程（避免显式胡克定律在刚度较大时系统发散/爆炸）。
    *   实现弹性悬挂与软连接表现。
*   **第10-11天：旋转/铰链关节 (Revolute / Pin Joint)**
    *   **第10天**：2自由度线位移约束（$p_2 - p_1 = \mathbf{0}$），将两个物体的特定锚点钉在一起，仅保留相对旋转自由度。
    *   **第11天（进阶）**：角度限制（Limit Angle，限制摆动范围）与关节马达（Motor，提供旋转动力，如车轮电机）。

#### 第4阶段：复合场景验证与工程收尾（第12-14天）
*   **第12天：综合物理用例构建 (Showcase Scenarios)**
    *   **布娃娃系统 (Ragdoll)**：使用 Capsule 制作四肢与躯干，使用带角度限制的 Revolute Joint 组装成小人。
    *   **避震小车 (Car Demo)**：主刚体车身 + Revolute/Spring Joint 连接圆球车轮。
    *   **吊桥/链条 (Suspension Bridge)**：10个小矩形依次用 Distance Joint 首尾相接。
*   **第13天：健壮性测试与调试导出**
    *   编写关节约束能量漂移测试：闭环四连杆机构持续运行 1000 帧，记录两端点位移误差。
    *   CSV 导出双摆（Double Pendulum）运动轨迹，验证混沌系统下的确定性表现。
*   **第14天：代码梳理、架构文档与 GitHub 发布**
    *   梳理 `include` 暴露的 Public API，统一工厂模式创建接口（如 `world.CreateJoint(...)`）。
    *   更新 README：加入关节物理公式推导、复杂场景配置说明以及事件监听的示例代码。

---

### V3 新增项目结构建议

```text
include/physics/
├── Dynamics/
│   ├── Material.h         # <--- 新增：材质定义与混合规则
│   └── Joints/            # <--- 新增：关节约束模块
│       ├── Joint.h        # 约束抽象基类
│       ├── DistanceJoint.h# 距离关节
│       ├── SpringJoint.h  # 弹簧关节（软约束）
│       └── RevoluteJoint.h# 旋转/铰链关节
├── Collision/
│   ├── Shapes/
│   │   ├── Shape.h        # 增加 GetMassData 纯虚函数
│   │   ├── Circle.h
│   │   ├── Capsule.h      # <--- 新增：胶囊体定义与计算
│   │   └── Polygon.h      # <--- 扩展：通用凸多边形与裁剪
├── Events/                # <--- 新增：事件总线系统
│   ├── ContactListener.h  # 碰撞监听接口 (Enter/Stay/Exit)
│   └── EventDispatcher.h  # 事件队列与分发器
```

---

### 这个 V3 版本在简历上的顶级亮点

1.  **高级约束力学系统**：
    *   "基于拉格朗日乘子法与 Sequential Impulses 实现了 Distance、Spring、Revolute 等多自由度约束解算器，并运用 Baumgarte 稳定化技术抑制关节位置漂移。"
2.  **软约束数学建模**：
    *   "推导并实现了基于频率与阻尼比参数的隐式弹簧阻尼关节，解决了高刚度下显式积分带来的物理爆炸问题。"
3.  **复杂接触流形解算**：
    *   "实现通用凸多边形的 Sutherland-Hodgman 裁剪算法，生成精准的多点接触流形（Manifold），确保多面体堆叠平稳无抖动。"
4.  **业务友好型引擎架构**：
    *   "设计基于位图与缓存差集的碰撞生命周期状态机，实现开箱即用的 `OnCollisionEnter/Stay/Exit` 触发器及事件队列安全分发机制。"

---

**建议你的启动步骤：**
第一步先创建 `Material.h`，并在 `Shape` 基类中引入材质指针，这是改动成本最小且后续所有模块都依赖的基础骨架。需要我为你提供一份 `Joint` 抽象基类或 `Material` 的头文件接口设计模板吗？