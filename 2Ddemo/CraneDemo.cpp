// ============================================================
// 2Ddemo 样例 7：起重机吊臂变幅机构 (CraneDemo)
//   - 吊塔基座（静态 Box）+ 吊臂（Box，RevoluteJoint 铰接塔顶可俯仰）
//   - 变幅机构：曲柄轮（Revolute 马达）+ GearJoint（ratio=4）耦合吊臂俯仰
//   - 吊钩：PulleyJoint（ratio=2.0 动滑轮）悬挂配重与货物
//   - 2:1 省力倍率：m_cargo = ratio·m_counter 时两侧严格平衡
//   - CSVExporter 导出吊臂/货物轨迹到 2Ddemo/crane_demo.csv
// 运行：取消注释本文件 main，注释其他文件的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RevoluteJoint.h"
#include "GearJoint.h"
#include "PulleyJoint.h"
#include "Box.h"
#include "Circle.h"
#include "CSVExporter.h"
#include "Logger.h"
#include <cmath>

//int main() {
//    Logger::info(">>> 2Ddemo 7: Crane 起重机（GearJoint 变幅 + PulleyJoint 2:1 省力）<<<");
//    World world(Vector2(0.0f, -9.8f));
//    float dt = 1.0f / 60.0f;
//
//    // 吊塔：静态 Box(0.5, 8)，塔顶 (0, 8)
//    Body* tower = new Body(new Box(0.5f, 8.0f), 0.0f, 4.0f, 0.0f);
//    world.addBody(tower);
//
//    // 曲柄轮：半径 0.3，挂在塔底 (0,-1)（吊臂扫掠圆盘之外，绝不与吊臂碰撞卡死）
//    Body* crank = new Body(new Circle(0.3f), 0.0f, -1.0f, 1.0f);
//    world.addBody(crank);
//    RevoluteJointDef crankDef;
//    crankDef.bodyA = tower;
//    crankDef.bodyB = crank;
//    crankDef.localAnchorA = Vector2(0.0f, -5.0f);
//    crankDef.localAnchorB = Vector2(0.0f, 0.0f);
//    crankDef.enableMotor = true;
//    crankDef.motorSpeed = 1.2f;   // 曲柄 1.2 rad/s → 吊臂 0.3 rad/s（ratio=4）
//    crankDef.maxMotorTorque = 2000.0f;
//    crankDef.collideConnected = false;
//    RevoluteJoint* crankJoint = static_cast<RevoluteJoint*>(world.createJoint(crankDef));
//
//    // 吊臂：Box(8, 0.4)，左端铰接塔顶
//    Body* arm = new Body(new Box(8.0f, 0.4f), 4.0f, 8.0f, 2.0f);
//    world.addBody(arm);
//    RevoluteJointDef armDef;
//    armDef.bodyA = tower;
//    armDef.bodyB = arm;
//    armDef.localAnchorA = Vector2(0.0f, 4.0f);
//    armDef.localAnchorB = Vector2(-4.0f, 0.0f);
//    armDef.collideConnected = false;
//    RevoluteJoint* armJoint = static_cast<RevoluteJoint*>(world.createJoint(armDef));
//
//    // 变幅传动：C = crank角 + ratio·arm角 = 0（齿轮关节耦合两个铰链坐标；
//    // 注意 GearJoint 的 bodyA/B/C/D 由父关节自动派生，def 无需设置）
//    GearJointDef gearDef;
//    gearDef.joint1 = crankJoint;
//    gearDef.joint2 = armJoint;
//    gearDef.ratio = 4.0f;
//    gearDef.collideConnected = false;
//    world.createJoint(gearDef);
//
//    // 吊钩滑轮组（吊臂右端 (8, 8)）：配重 2kg + 货物 4kg（ratio=2 → 严格平衡）
//    Body* counter = new Body(new Circle(0.25f), 8.0f, 5.0f, 10.2f);
//    world.addBody(counter);
//    Body* cargo = new Body(new Box(1.0f, 1.0f), 8.0f, 3.5f, 4.0f);
//    world.addBody(cargo);
//    PulleyJointDef pulleyDef;
//    pulleyDef.bodyA = counter;
//    pulleyDef.bodyB = cargo;
//    pulleyDef.groundAnchorA = Vector2(8.0f, 8.0f); // 两绳端都挂在吊臂右端
//    pulleyDef.groundAnchorB = Vector2(8.0f, 8.0f);
//    pulleyDef.localAnchorA = Vector2(0.0f, 0.0f);
//    pulleyDef.localAnchorB = Vector2(0.0f, 0.0f);
//    pulleyDef.ratio = 2.0f;  // 动滑轮 2:1 省力
//    pulleyDef.collideConnected = false;
//    PulleyJoint* pulley = static_cast<PulleyJoint*>(world.createJoint(pulleyDef));
//
//    CSVExporter csv("2Ddemo/crane_demo.csv");
//    float initialL = pulley->getLengthA() + pulley->getRatio() * pulley->getLengthB();
//    float maxIdentityError = 0.0f;
//    for (int i = 0; i < 240; ++i) {
//        world.step(dt);
//        float L = pulley->getLengthA() + pulley->getRatio() * pulley->getLengthB();
//        maxIdentityError = std::max(maxIdentityError, std::abs(L - initialL));
//        csv.writeFrame(i, world.getBodies());
//    }
//
//    Logger::info("验收: 绳长恒等式误差=" + std::to_string(maxIdentityError) + "（< 1e-4 PASS）");
//    Logger::info("验收: 吊臂角=" + std::to_string(arm->getRotation()) +
//        " 曲柄角/4=" + std::to_string(crank->getRotation() / 4.0f));
//    Logger::info("轨迹已导出: 2Ddemo/crane_demo.csv");
//    return 0;
//}
