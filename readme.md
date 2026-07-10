# MyPhysicsEngine2D
一个基于 C++11 构建的高仿真、纯逻辑 2D 刚体物理引擎。
## 📌 项目愿景
本项目旨在不依赖任何图形库（如 OpenGL/SDL）的情况下，从 0 到 1 手搓一套工业级物理模拟逻辑。通过数据驱动和单元测试验证物理公式的准确性。
- **核心目标**：实现动量守恒、能量损耗、摩擦力、转动惯量以及稳定的多体堆叠。
- **验证方式**：控制台实时日志、CSV 数据导出、Excel 轨迹分析。
---
## 🛠 项目结构
```text
MyPhysicsEngine2D/
├── docs/               # 物理公式推导与设计文档
├── include/            # 头文件 (.h)
│   └── physics/
│       ├── Common/     # 基础数学库 (Vector2, Settings)
│       ├── Collision/  # 碰撞几何体与检测 (Shape, Circle, Box, Manifold) <-- 更新
│       ├── Dynamics/   # 动力学核心 (Body, World)
│       └── Utils/      # 工具类 (CSVExporter)
├── src/                # 源代码 (.cpp)
│   ├── Collision/      # 碰撞检测算法实现 (Collision.cpp)            <-- 更新
│   ├── Dynamics/       
│   └── ...
├── tests/              # 单元测试与 Demo 入口
│   └── CollisionTests.cpp # Day 04 碰撞测试脚本                    <-- 新增
├── output/             # 模拟生成的 CSV 数据文件
├── scripts/            # Python 可视化脚本 (Matplotlib)
└── README.md
```
---
## 📅 进度跟踪 (15天挑战)
### 第 1 阶段：核心数学与基础结构
- [x] **Day 01: Vector2 数学库**
  - 实现基础运算符重载 (`+`, `-`, `*`, `/`)。
  - 实现点积 (`Dot`)、2D 叉积 (`Cross`)、归一化 (`Normalize`)。
  - 实现向量旋转 (`Rotate`) 及垂直向量计算。
  - 引入 `EPSILON` 处理浮点数精度问题。
- [x] **Day 02: 物理状态与积分器**
  - 构建 `RigidBody` 类（位置、速度、力、质量）。
  - 实现 `World` 物理步进与 **半隐式欧拉 (Semi-implicit Euler)** 积分。
  - 实现 **CSV 数据导出系统** 与 Python 可视化。
- [x] **Day 03: 形状定义与旋转动力学 (Shape & Rotation)**
  - 实现 `Shape` 基类与 `Circle`, `Box` 子类。
  - 引入旋转状态量：角度、角速度、转矩、转动惯量。
  - 实现偏心力产生转矩的计算逻辑。
  - 完成平动与转动的耦合模拟。
### 第2阶段：碰撞检测
- [x] **Day 04: 基础碰撞检测与流形构造 (Circle vs. Circle)**
  - 实现 `Manifold` 结构，存储法线、穿透深度与接触点。
  - 实现圆与圆的碰撞判定算法。
  - 解决圆心完全重合时的除零异常（NaN Normal）。
  - 通过单元测试验证边界情况（相切、重叠、同心）。
---
## 🚀 Day 01进展：Vector2 核心库
### 1. 技术选型
- **浮点数处理**：使用 `float` 权衡性能与精度，定义 `EPSILON = 0.0001f` 用于相等判断。
- **性能优化**：
    - 关键数学函数（如 `LengthSquared`）采用 `inline` 定义。
    - 归一化函数采用“一次除法、两次乘法”的策略。
    - 严格遵循 `const` 正确性，优化参数传递。
### 2. 核心公式实现
- **旋转公式**：
  $$x' = x \cos\theta - y \sin\theta$$
  $$y' = x \sin\theta + y \cos\theta$$
- **安全归一化**：
  在计算 $1/length$ 前检查长度是否大于 `EPSILON`，彻底杜绝 `NaN` 崩溃问题。
### 3. 如何验证
运行 `tests/VectorTests.cpp`。当前已通过以下测试：
- ✅ 向量基础加减法
- ✅ 标量乘法交换律 (`vec * 2.0` 与 `2.0 * vec`)
- ✅ 长度与距离计算（3-4-5 三角形验证）
- ✅ 90度旋转与正交性校验
- ✅ 零向量归一化安全检查
## 🚀 Day 02进展：动力学与积分器

### 1. 核心算法：半隐式欧拉法 (Semi-implicit Euler)
为了保证物理模拟的能量守恒与稳定性，弃用了显式欧拉法，采用了工业级标准的半隐式欧拉积分：
1.  **更新加速度**： $\vec{a} = \frac{\sum\vec{F}}{m} + \vec{g}$
2.  **更新速度**： $\vec{v}_{new} = \vec{v}_{old} + \vec{a} \cdot \Delta t$
3.  **更新位置**： $\vec{p}_{new} = \vec{p}_{old} + \vec{v}_{new} \cdot \Delta t$

### 2. 关键工程优化
- **质量倒数 (Inverse Mass)**：在 `Body` 类中存储 `invMass` ($1/m$)。
    - 解决了静态物体（地面、墙壁）质量无穷大的表示问题（$invMass = 0$）。
    - 将除法运算转化为乘法运算，提升 `Step` 函数执行效率。
- **数据驱动验证**：开发了 `CSVExporter` 工具，将物理步进数据导出为标准 CSV 格式。

### 3. 可视化验证
通过 Python (Matplotlib) 脚本对导出的数据进行了重绘，成功验证了以下物理现象：
- ✅ **水平匀速运动**：在无水平外力下，$X$ 轴坐标呈线性增长。
- ✅ **自由落体/抛物线**：$Y$ 轴速度随时间线性递减，位移符合二次函数规律。
- ✅ **数据精度**：经对比，模拟轨迹与理论解析解误差在 $0.1\%$ 以内。
## 🚀 Day 03进展：形状定义与旋转动力学

### 1. 形状与转动惯量 (Moment of Inertia)
为了实现真实的物理旋转，不再将物体视为单纯的质点，而是具有几何属性的刚体：
- **类型识别**：采用 `enum` + `Inheritance` 的混合方案。基类 `Shape` 提供 `Type` 标签用于快速类型判断，提供虚函数 `ComputeMass` 处理多态计算。
- **惯量计算**：为不同形状实现了转动惯量公式，确保物体的旋转行为与其形状、大小、质量分布相符合：
    - **圆形**：$I = \frac{1}{2}mr^2$
    - **矩形**：$I = \frac{1}{12}m(w^2 + h^2)$
- **静态属性**：引入 `invInertia` (惯量倒数)，当 $invInertia = 0$ 时，物体在物理上表现为无法被旋转的固定体。

### 2. 旋转动力学积分
在 `World::Step` 中扩展了积分器，使其支持角动量更新：
1.  **角加速度**：$\alpha = \frac{\sum\tau}{I}$ (转矩 / 转动惯量)
2.  **角速度**：$\omega_{new} = \omega_{old} + \alpha \cdot \Delta t$
3.  **角度**：$\theta_{new} = \theta_{old} + \omega_{new} \cdot \Delta t$

### 3. 偏心力与转矩 (Eccentric Force)
实现了 `ApplyForceAtPoint` 接口，模拟力作用于非质心位置产生的物理效果：
- **力矩合成**：$\vec{\tau} = \vec{r} \times \vec{F}$ (其中 $\vec{r}$ 是从质心到作用点的位移向量)。
- **物理效果**：当对矩形的顶点施加水平力时，物体在沿抛物线坠落的同时，会根据冲量产生持续的匀速旋转，完美还原了现实中的“翻滚”现象。
---
### 4. 验证结果
通过 CSV 导出数据与 Python 可视化脚本验证：
- ✅ **动量守恒**：在无后续外力情况下，角速度 $\omega$ 保持恒定。
- ✅ **几何正确性**：矩形与圆形的旋转速度差异符合转动惯量公式预想。
- ✅ **可视化**：生成的轨迹图中，紫色向量（方向示意）随时间平滑转动，证明旋转逻辑正确。
---
## 🚀 Day 04进展：碰撞检测与流形 (Manifold)

### 1. 碰撞流形的设计
为了让后续的冲量解算器（Solver）能够知道如何修补物理穿透，我们实现了 `Manifold` 结构。它不仅记录“是否撞了”，还记录了关键的几何数据：
- **Normal (碰撞法线)**：从物体 A 指向 B 的单位向量，定义了排斥力的方向。
- **Penetration (穿透深度)**：两个圆重叠的距离，用于位置修正（Position Correction）。
- **Contact Points (接触点)**：力作用的具体坐标，对于旋转产生的转矩计算至关重要。

### 2. 圆 vs 圆 检测算法
通过比较两圆心欧几里得距离 $d$ 与半径之和 $R_1 + R_2$ 进行判定。
- **鲁棒性处理**：
  针对**同心圆（$d=0$）**的情况，通过手动强制指定法线 `(0, 1)`，避免了归一化导致的 `NaN` 崩溃问题。

### 3. 如何验证
运行 `tests/CollisionTests.cpp`。当前已通过以下严苛测试用例：
- ✅ **Far Away**: 距离大于半径和，判定为无碰撞。
- ✅ **Overlapping**: 水平重叠 0.5 单位，法线准确识别为 `(1, 0)`，穿透深度 `0.5`。
- ✅ **Just Touching**: 距离刚好等于半径和，识别为临界碰撞（相切）。
- ✅ **Concentric**: 圆心完全重合，触发安全保护逻辑，输出默认法线。

**测试输出示例：**
```text
==== Test: Overlapping (Horizontal) ====
Result: COLLISION!
  Penetration: 0.5
  Normal:      (1, 0)
  Contact Pt:  (1, 0)
```

## 💻 编译与运行
1. 使用 **Visual Studio 2019/2022** 打开解决方案。
2. 确保项目属性中 `Additional Include Directories` 包含 `$(ProjectDir)include`。
3. 设置 C++ 标准为 **ISO C++11**。
---
