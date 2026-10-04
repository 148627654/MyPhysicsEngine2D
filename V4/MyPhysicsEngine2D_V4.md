这份 **V4 约束全家桶与交互查询版（Constraint Mastery & Query Edition）** 14 天开发进度表，专门针对 V3 之后引擎还缺的三块能力进行编排：**关节家族补全**（V3 只做了 Distance/Spring/Revolute 三类，还剩 8 类经典关节）、**世界级查询 API**（射线/区域/点查询）、**Bullet 连续碰撞打通**（`m_isBullet` 目前只是标记，TOI 基础设施 V2 已写但没接入高速物体流程）。

V3 建立的关节框架（`Joint` 基类三管线 + `collideConnected` 过滤 + Island DFS 自动并入 + `World::createJoint` 工厂）就是为这一天准备的——每个新关节只需实现三个虚函数，不用再动求解器骨架。

---

# 14天开发进度表

#### 第1阶段：锁定与平移类关节（第1-4天）

*   **第1天：焊接关节 (WeldJoint)**
    *   约束方程：$C = (p_2 - p_1, \theta_2 - \theta_1 - \theta_{ref}) = \mathbf{0}$，3 自由度全锁定。
    *   **实现要点**：K 矩阵 3×3（线 × 线、线 × 角、角 × 角三个块）；Revolute 等角状态机的直接推广——Revolute 锁 1 个角度，Weld 锁 2 个平移 + 1 个角度。
    *   **验证**：悬臂梁（一端焊接在地面锚点，另一端挂重物）任意角度不掉不转；`collideConnected=false` 时焊接物体互相不产生实体接触。
*   **第2天：滑块关节骨架 (PrismaticJoint)**
    *   数据：滑轨轴 $t$（世界系）与垂轴 $n$；速度雅可比含质心沿 $t$ 的平移 + 相对旋转两个自由度（K 矩阵 2×2）。
    *   **平移限位**：与 Revolute 限位状态机**同构**（Inactive/AtLower/AtUpper/Equal），把角度 C 换成沿轴位移 $C = t \cdot (p_B - p_A) - x_{ref}$ 即可，Baumgarte 偏置同款 $-0.2\,C/dt$。
    *   **验证**：滑块在轨道上只滑不转、限位处反弹（超界 < 0.005 m）。
*   **第3天：滑块马达 (Prismatic Motor)**
    *   轴马达：$\pm dt \cdot \tau_{max}$ 双向饱和、与限位冲量分开记账（V3 Revolute 马达同款防污染）。
    *   同天加装 **FrictionJoint（摩擦关节）**：线/角摩擦约束，冲量上限截断 $\lambda \in [-\mu \cdot P, +\mu \cdot P]$——有界冲量是全新的一课，和 V3 所有关节的"无界 λ"都不同。
    *   **验证**：电梯间匀速巡航、静摩擦挂住斜面物体。
*   **第4天：绳关节 (RopeJoint)**
    *   **不等式约束**：$C = \|p_B - p_A\| - L \le 0$（最大距离，不是固定距离）。
    *   **实现要点**：$C < 0$ 时整帧不施加（绳子松弛）；接触冲击时 $\lambda \ge 0$ 单侧截断——这是本迭代第一个不等式约束，与 DistanceJoint 的双侧刚性区别开。
    *   **验证**：链锤绕桩甩动，绳子永远不长于 L、松弛段自然下垂。

#### 第2阶段：复合传动与交互关节（第5-8天）

*   **第5天：滑轮关节 (PulleyJoint)**
    *   约束：两刚体锚点 $p_A, p_B$ + 两固定地面点 $g_A, g_B$，长度恒等式 $L = \|p_A - g_A\| + ratio \cdot \|p_B - g_B\|$。
    *   **实现要点**：B 侧分母质量按 $ratio^2$ 缩放（$m_{eff} = m_A + ratio^2 m_B$）——滑轮的"省力费距离"全在这个 ratio 里。
    *   **验证**：吊臂升降 ratio=2 时 B 移动距离是 A 的一半、受力是一倍。
*   **第6天：齿轮关节 (GearJoint)**
    *   约束：耦合两个既有关节（Revolute±Prismatic）的坐标 $C = \theta_A \pm ratio \cdot x_B$，本质是"关节的关节"。
    *   **实现要点**：持有两个 `Joint*` 引用 + 销毁通知——被引用的关节销毁时齿轮自动失效（V3 级联销毁已铺路，补一个关节引用检查即可）。
    *   **验证**：两个啮合齿轮反向转动，$\omega_B / \omega_A = -1/ratio$ 精确成立。
*   **第7天：车轮关节 (WheelJoint)**
    *   约束：质心沿悬挂轴的线约束（投影距离 + 相对角，2×2 K）+ 角马达——"硬悬挂"：没有弹簧，轮距刚性锁定。
    *   **与 V3 CarDemo 对比**：V3 用 SpringJoint（软悬挂）+ Revolute（马达）拼车，WheelJoint 是它的"刚性版"，过减速带时会颠（正常），平路上更稳。
    *   **验证**：战车以恒定马达角速度爬上 V3 同款 15° 坡，轮子不脱轴。
*   **第8天：鼠标关节 (MouseJoint)**
    *   **软目标点约束**：弹簧参数化 $\gamma = 1/(h(\beta + h\gamma))$ 由 frequencyHz/dampingRatio 推出（与 SpringJoint 软分支共享推导），冲量上限 $\lambda_{max}$ 限幅防猛拉爆。
    *   **交互意义**：这是引擎第一个"用户实时输入"关节，为 V7 编辑器拖拽铺路。
    *   **验证**：程序模拟鼠标拾取刚体 → 拖拽绕桩 → 松手飞出，全程无抖动、拉不爆。

#### 第3阶段：世界查询与连续碰撞（第9-11天）

*   **第9天：世界级查询 API**
    *   `World::rayCast(p1, p2, callback)`：封装已有 DynamicTree/BroadPhase 的回调底子（`rayCast` 已在树层实现），输出 `RayCastHit{ body, point, normal, fraction }`，支持过滤回调提前返回、最近命中。
    *   `World::queryAABB(aabb, callback)`：区域查询（引爆范围、视野、传感器扫描）。
    *   点查询：形状 `TestPoint` + 世界拾取（点选刚体）。
    *   **验证**：射线穿过多个物体只报最近者、trigger 可被过滤；V3 Polygon 射线测试（t=0.32987）回归不变。
*   **第10天：Bullet 连续碰撞 (CCD)**
    *   把 `Body::m_isBullet` 真正接进步进：bullet 刚体的宽相 AABB 按本帧位移扫掠扩展（swept AABB）。
    *   复用 V2 留下的 `TimeOfImpact` + `solveTOI`：对候选对求最早 TOI → 回溯到碰撞时刻 → 施加冲量 → 推进剩余时间。
    *   **实现要点**：只有 bullet 参与的接触对才走 TOI 路径，普通物体零额外开销（性能回归不许掉）。
*   **第11天：高速验收**
    *   子弹 100 m/s 打 0.1 m 薄墙：开 CCD 命中率 100%，关 CCD 必穿（对比验证 CCD 有效性）。
    *   子弹打 45° 斜墙弹道符合反射定律；1000 球落塔回归通过（TOI 不干扰常规求解路径）。

#### 第4阶段：复合场景与工程收尾（第12-14天）

*   **第12天：样例 1-2：电梯与起重机**
    *   **电梯 (ElevatorDemo)**：PrismaticJoint 马达 + 上下限位，模拟楼层停靠（马达启停、限位缓冲），乘客刚体随行。
    *   **起重机 (CraneDemo)**：PulleyJoint 吊臂升降 + GearJoint 变幅机构，吊起 10 倍自身重量的负载，绳长恒等式始终成立。
*   **第13天：样例 3-4：战车与链锤**
    *   **战车 (TankDemo)**：WheelJoint×2 + GearJoint 传动 + Revolute 炮塔马达，重跑 V3 金币坡道——对比 SpringJoint 拼车方案写一段"软悬挂 vs 硬悬挂"结论。
    *   **链锤 (FlailDemo)**：RopeJoint 甩链锤砸墙（验证 λ≥0 冲击行为）+ MouseJoint 拖拽交互演示（程序化鼠标轨迹）。
*   **第14天：全量回归与发布**
    *   GoogleTest 36 用例全绿 + 新增 V4 测试（每类关节 1 个最小验证场景，走 V3 确立的四步流程：原版 → 移植 → 注册 vcxproj → pre-commit 自动跑）。
    *   旧 2Ddemo 五样例（Ragdoll/Car/Bridge/ClosedLoop/DoublePendulum）回归全绿。
    *   更新 `docs/PublicAPI.md`（查询 API 表 + 8 类新关节 API 表）、README 增补传动场景说明、提交推送 GitHub。

---

### V4 新增项目结构建议

```text
include/physics/
├── Dynamics/
│   ├── WeldJoint.h        # <--- 新增：焊接关节（3 自由度锁定）
│   ├── PrismaticJoint.h   # <--- 新增：滑块关节（限位+马达）
│   ├── FrictionJoint.h    # <--- 新增：摩擦关节（有界冲量）
│   ├── RopeJoint.h        # <--- 新增：绳关节（不等式约束）
│   ├── PulleyJoint.h      # <--- 新增：滑轮关节（传递比）
│   ├── GearJoint.h        # <--- 新增：齿轮关节（关节耦合）
│   ├── WheelJoint.h       # <--- 新增：车轮关节（硬悬挂+马达）
│   └── MouseJoint.h       # <--- 新增：鼠标关节（软目标点）
├── Collision/
│   └── RayCastHit.h       # <--- 新增：查询结果结构
└── Events/
    └── QueryCallback.h    # <--- 新增：rayCast/queryAABB 回调接口
```

> 目录保持 V3 实际的 Dynamics/ 平铺风格（V3 建议的 Joints/ 子目录当时未采用，V4 延续平铺，避免移动既有文件）。

---

### 这个 V4 版本在简历上的顶级亮点

1.  **完整关节约束家族**：
    *   "实现 Weld/Prismatic/Friction/Rope/Pulley/Gear/Wheel/Mouse 8 类关节，覆盖焊接、滑块限位马达、滑轮倍率、齿轮传动、硬悬挂与交互拖拽，关节族完整度对齐 Box2D。"
2.  **不等式与有界约束求解**：
    *   "为绳关节实现 λ≥0 单侧截断、为摩擦/鼠标关节实现 ±μP 冲量饱和，掌握拉格朗日乘子法的约束投影（Constraint Projection）处理。"
3.  **世界级空间查询**：
    *   "基于动态 AABB 树封装 RayCast/QueryAABB/点查询三套查询接口，支持过滤回调与最近命中，为拾取、视野、引爆等游戏系统提供 O(log N) 底层支持。"
4.  **连续碰撞检测 (CCD)**：
    *   "打通 Bullet 连续碰撞管线：扫掠 AABB + TOI 回溯子步，100 m/s 子弹对 0.1 m 薄墙命中率 100%，普通物体零额外开销。"

---

**建议你的启动步骤：**
第一步先写 `WeldJoint.h/.cpp`——它是 V3 Revolute 等角状态机的直接推广（1 自由度 → 3 自由度），复用 `Joint` 基类三管线、`collideConnected` 过滤和工厂 `createJoint`，改动面最小，能立刻验证 V3 关节框架的可扩展性是否过关。需要我为你提供 `WeldJoint` 的头文件接口设计模板吗？
