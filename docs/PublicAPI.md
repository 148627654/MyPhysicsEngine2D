# MyPhysicsEngine2D 公开 API 文档

> 引擎入口：`#include "physics/Dynamics/World.h"`（其余头文件按需包含）

## 0. 命名规范

| 类别 | 规范 | 示例 |
|---|---|---|
| 类 / 结构体 / 枚举 | UpperCamelCase | `World`、`Body`、`RevoluteJoint`、`ContactListener` |
| 函数 / 方法 | lowerCamelCase | `step()`、`createBody()`、`getPosition()` |
| 成员变量 | m_ + lowerCamelCase | `m_bodies`、`m_impulse` |
| 常量 | UPPER_SNAKE / 命名空间聚合 | `Settings::PAI`、`Settings::DT` |

**创建对象的唯一推荐方式**：`World` 的工厂方法（见 §2），不直接 `new`。

---

## 1. 目录结构与头文件一览

```
include/physics/
├── Common/
│   ├── Vector2.h       二维向量（数学基础）
│   └── Setting.h       引擎常量（Settings 命名空间）
├── Collision/
│   ├── Shape.h         形状基类（材质 + 触发器 + 质量）
│   ├── Box.h / Circle.h / Capsule.h / Polygon.h   具体形状
│   ├── Collision.h     窄相检测（SAT / 最近点 / 射线）
│   ├── Contact.h       接触（接触流形 + 触发器判定）
│   ├── AABB.h          轴对齐包围盒
│   ├── BroadPhase.h    宽相（AABB 树）
│   └── TimeOfImpact.h  TOI（连续碰撞）
├── Dynamics/
│   ├── World.h         物理世界（总入口、工厂）
│   ├── Body.h          刚体
│   ├── Material.h      物理材质（Physics2D::Material）
│   ├── Joint.h         关节基类 + JointType
│   ├── DistanceJoint.h 距离关节（刚性杆 / 弹性弹簧）
│   ├── RevoluteJoint.h 旋转关节（销钉 + 角度限位 + 马达）
│   ├── SpringJoint.h   弹簧关节
│   ├── Island.h        岛屿求解器（诊断用）
│   └── ContactManager.h 接触生命周期管理
├── Events/
│   ├── ContactListener.h 事件回调接口（推送式）
│   ├── CollisionEvent.h  碰撞/触发器事件数据
│   └── ContactKey.h      接触键与记录（ContactRecord）
└── Utils/
    ├── Logger.h        日志
    ├── CSVExporter.h   轨迹导出
    └── Profiler.h      性能分析
```

---

## 2. 工厂接口（World）

### 2.1 刚体工厂

```cpp
World world(Vector2(0, -9.8f));          // 重力

// 静态地面（density = 0 → 静态）
world.createBox(20.0f, 2.0f, 0.0f, -2.0f, 0.0f);

// 动态刚体（density > 0 → 动态，质量 = 密度 × 面积）
Body* ball  = world.createCircle(0.5f, 0.0f, 5.0f, 1.0f);
Body* box   = world.createBox(1.0f, 1.0f, 3.0f, 5.0f, 2.5f);
Body* cap   = world.createCapsule(0.5f, 2.0f, -3.0f, 5.0f, 1.0f);

// 自定义形状 / 材质
Physics2D::Material mat;
mat.density = 2.0f; mat.restitution = 0.8f; mat.dynamicFriction = 0.6f;
Body* custom = world.createCircle(1.0f, 0.0f, 8.0f, 1.0f, mat);
custom->getShape()->isTrigger = true;   // 设为触发器（无物理响应）
```

**工厂签名**：

| 方法 | 说明 |
|---|---|
| `Body* createBody(Shape*, x, y, density)` | 用现成形状创建刚体 |
| `Body* createBox(w, h, x, y, density, mat)` | 矩形 |
| `Body* createCircle(radius, x, y, density, mat)` | 圆形 |
| `Body* createCapsule(radius, length, x, y, density, mat)` | 胶囊 |
| `Joint* createJoint(const JointDef&)` | 按 `def.type` 分发（Distance / Spring / Revolute） |

### 2.2 关节工厂示例

```cpp
// 销钉：把摆臂钉在静态锚点上（用 def.initialize 自动算局部锚点）
RevoluteJointDef def;
def.initialize(anchor, arm, Vector2(0.0f, 5.0f));  // 世界锚点
def.enableLimit  = true;
def.lowerAngle   = -0.785f;   // -45°
def.upperAngle   =  0.785f;   // +45°
def.enableMotor  = true;
def.motorSpeed   = 10.0f;
def.maxMotorTorque = 100.0f;
RevoluteJoint* j = static_cast<RevoluteJoint*>(world.createJoint(def));
```

---

## 3. 主循环与对象生命周期

```cpp
for (int i = 0; i < 600; ++i) {
    world.step(1.0f / 60.0f);   // 固定步长主循环
}

// 延迟销毁：在接触回调内调用也安全（本帧 step 末尾真正移除）
world.destroyBody(ball);

// 关节：销毁刚体时级联销毁相连关节；也可单独销毁
world.destroyJoint(joint);
```

---

## 4. Body 常用 API

| 方法 | 说明 |
|---|---|
| `getPosition() / setPosition(x, y)` | 位置 |
| `getRotation() / setRotation(r)` | 角度（弧度） |
| `getVelocity() / setVelocity(v)` | 线速度 |
| `getAngularVelocity() / setAngularVelocity(w)` | 角速度 |
| `addForce(f) / applyImpulse(p) / applyForceAtPoint(f, pt) / addTorque(t)` | 力与冲量 |
| `getMass() / getInertia() / getInvMass() / getInvInertia()` | 质量属性 |
| `getRestitution() / setRestitution(e)` | 恢复系数（透传 shape->material） |
| `getFriction() / setFriction(f)` | 摩擦系数（透传） |
| `setGravityScale(g)` | 重力缩放 |
| `setAwake(w) / isAwake() / setSleepAllowed(a) / forceSleep()` | 睡眠控制 |
| `setBullet(b) / isBullet()` | CCD 高速标记 |
| `getShape()` | 形状指针（访问 material / isTrigger） |
| `updateMassData()` | 运行时改密度后重算质量 |

---

## 5. 关节 API

### 5.1 DistanceJoint（刚性杆 / 弹性弹簧）

```cpp
DistanceJointDef def;
def.bodyA = a; def.bodyB = b;
def.length = 5.0f;              // 杆长
// 弹性弹簧模式（frequencyHz > 0 时启用，隐式积分无条件稳定）：
def.frequencyHz = 2.0f;         // 固有频率
def.dampingRatio = 0.0f;        // 阻尼比（1 = 临界阻尼）
def.enableWarmStart = true;     // 热启动
```

### 5.2 RevoluteJoint（销钉 + 限位 + 马达）

| 成员 / 方法 | 说明 |
|---|---|
| `def.initialize(bA, bB, worldAnchor)` | 按世界锚点装配（自动算局部锚点与基准角） |
| `def.enableLimit / lowerAngle / upperAngle` | 角度限位（等角 → 刚性焊接） |
| `def.enableMotor / motorSpeed / maxMotorTorque` | 马达（恒速 + 扭矩钳位） |
| `getJointAngle()` | 当前相对角度 θB − θA − θref |
| `getMotorTorque(dt)` | 本帧马达等效扭矩 |
| `setMotorSpeed / setMaxMotorTorque / enableMotor / enableLimit` | 运行时控制 |

---

## 6. 接触事件（推送式回调）

```cpp
class MyListener : public ContactListener {
    void onCollisionEnter(const CollisionEvent& e) override {
        // e.bodyA / e.bodyB / e.normal / e.contacts
    }
    void onTriggerEnter(const TriggerEvent& e) override {
        // e.triggerBody / e.otherBody
    }
};
world.setContactListener(&listener);
```

**轮询式**（不注册监听器时用）：`world.getContactManager().getLifecycleRecords()` 返回本帧全部 `ContactRecord`（含 Enter/Stay/Exit 状态与 isTrigger 标记）。

---

## 7. 查询与诊断

| 方法 | 说明 |
|---|---|
| `world.rayCast(p1, p2)` | 射线检测（打印命中信息） |
| `world.getBodies()` | 所有刚体 |
| `world.getIslands()` | 本帧岛屿（`getBodyCount() / getJointCount()` 检查连通性） |
| `world.getJointCount()` | 关节数量 |
| `world.getContactMap()` | 活跃接触对 |
| `world.getProfiler()` | 性能分析器（`printReport()`） |
| `Logger::info(...)` | 日志输出 |

---

## 8. 常见陷阱（历史踩坑记录）

1. **静态刚体不会移动也不会被宽相重新同步**——瞬移静态刚体后其代理不会更新，接触不会被发现。
2. **形状重叠会触发睡眠系统的速度清零**——铰链连接的刚体若与锚点载体重叠，会被冻结（速度每帧归零）。锚点载体请设 `isTrigger`。
3. **竖直/水平对称摆放 = 不稳定平衡**——重力矩严格为零，数值上永不倒下；测试时给微小初始扰动。
4. **下落加速期的低速阶段会被睡眠系统冻结**——需要 `setSleepAllowed(false)` 的测试务必设置。
5. **`destroyBody` 是延迟销毁**——回调内可安全调用；`removeBody` 是立即销毁（避免在回调中使用）。
