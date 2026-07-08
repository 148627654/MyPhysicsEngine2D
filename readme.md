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
│       ├── Common/     # 基础数学库
│       ├── Dynamics/   # 动力学核心 (Body, World)
│       └── Utils/      # 工具类 (CSVExporter)
├── src/                # 源代码 (.cpp)
├── tests/              # 单元测试代码
├── output/             # 模拟生成的 CSV 数据文件
├── scripts/            # Python 可视化脚本
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
- [ ] **Day 03: 形状定义 (Collider)**
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
1.  **更新加速度**：$\vec{a} = \frac{\sum\vec{F}}{m} + \vec{g}$
2.  **更新速度**：$\vec{v}_{new} = \vec{v}_{old} + \vec{a} \cdot \Delta t$
3.  **更新位置**：$\vec{p}_{new} = \vec{p}_{old} + \vec{v}_{new} \cdot \Delta t$

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
---
## 💻 编译与运行
1. 使用 **Visual Studio 2019/2022** 打开解决方案。
2. 确保项目属性中 `Additional Include Directories` 包含 `$(ProjectDir)include`。
3. 设置 C++ 标准为 **ISO C++11**。
---


