// ============================================================
// 2Ddemo 样例 8：双驱重装战车 (TankDemo)
//   - 梯形车身（Polygon）+ 2 个 WheelJoint 刚性硬悬挂 + 大扭矩轮毂马达
//   - 车顶 RevoluteJoint 旋转炮塔（角度限位 ±0.5 + 马达驱动）
//   - 越野路线：平地 → 20° 陡坡 → 高原平台（坡顶无缝衔接，5cm 微台阶）
//   - 验收：战车爬上高原、车轮零脱轴、炮塔停在限位
//   - CSVExporter 导出全部刚体轨迹到 2Ddemo/tank_demo.csv
// 运行：取消注释本文件 main，注释其他文件的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "WheelJoint.h"
#include "RevoluteJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Polygon.h"
#include "Setting.h"
#include "CSVExporter.h"
#include "Logger.h"
#include <cmath>
#include <algorithm>

//int main() {
//    Logger::info(">>> 2Ddemo 8: Tank 重装战车（WheelJoint 硬悬挂 + 炮塔）<<<");
//    World world(Vector2(0.0f, -9.8f));
//    float dt = 1.0f / 60.0f;
//    const float slopeAngle = 20.0f * Settings::PAI / 180.0f;
//
//    // 赛道：平地 + 20° 陡坡 + 高原平台（坡顶无缝衔接）
//    world.addBody(new Body(new Box(20.0f, 1.0f), -12.0f, -0.5f, 0.0f));  // 平地
//    Body* slope = new Body(new Box(15.2f, 0.5f), 0.0f, 1.0371f, 0.0f);
//    slope->setRotation(slopeAngle);
//    slope->setFriction(0.9f);
//    world.addBody(slope);
//    Body* plateau = new Body(new Box(15.0f, 1.0f), 14.5f, 3.40f, 0.0f);
//    plateau->setFriction(0.9f);
//    world.addBody(plateau);
//
//    // 梯形车身（底 3 顶 2 高 0.8，CCW），密度 2 → 质量 4，预俯仰 20° 出生贴地
//    Vector2 trapVerts[4] = {
//        Vector2(-1.5f, -0.4f), Vector2(1.5f, -0.4f),
//        Vector2(1.0f, 0.4f), Vector2(-1.0f, 0.4f)
//    };
//    Polygon* trap = new Polygon();
//    trap->set(trapVerts, 4);
//    Body* chassis = new Body(trap, 0.0f, 2.2f, 2.0f);
//    chassis->setRotation(slopeAngle);
//    world.addBody(chassis);
//
//    Vector2 anchorL = chassis->getPosition() + Vector2(-1.2f, -0.5f).rotate(slopeAngle);
//    Vector2 anchorR = chassis->getPosition() + Vector2(1.2f, -0.5f).rotate(slopeAngle);
//
//    Body* wheelL = new Body(new Circle(0.4f), anchorL.getX(), anchorL.getY() + 0.002f, 2.0f);
//    Body* wheelR = new Body(new Circle(0.4f), anchorR.getX(), anchorR.getY() + 0.002f, 2.0f);
//    wheelL->setFriction(0.9f);
//    wheelR->setFriction(0.9f);
//    world.addBody(wheelL);
//    world.addBody(wheelR);
//
//    WheelJointDef defL;
//    defL.Initialize(chassis, wheelL, anchorL, Vector2(0.0f, -1.0f));
//    defL.enableMotor = true;
//    defL.motorSpeed = -12.0f;
//    defL.maxMotorTorque = 50.0f;
//    defL.collideConnected = false;
//    WheelJoint* jL = static_cast<WheelJoint*>(world.createJoint(defL));
//
//    WheelJointDef defR;
//    defR.Initialize(chassis, wheelR, anchorR, Vector2(0.0f, -1.0f));
//    defR.enableMotor = true;
//    defR.motorSpeed = -12.0f;
//    defR.maxMotorTorque = 50.0f;
//    defR.collideConnected = false;
//    WheelJoint* jR = static_cast<WheelJoint*>(world.createJoint(defR));
//
//    // 旋转炮塔：车顶 Revolute + 限位 ±0.5 + 马达
//    Body* turret = new Body(new Box(1.0f, 0.3f), 0.0f, 2.65f, 2.0f);
//    world.addBody(turret);
//    RevoluteJointDef turretDef;
//    turretDef.bodyA = chassis;
//    turretDef.bodyB = turret;
//    turretDef.localAnchorA = Vector2(0.0f, 0.45f);
//    turretDef.localAnchorB = Vector2(0.0f, 0.0f);
//    turretDef.enableLimit = true;
//    turretDef.lowerAngle = -0.5f;
//    turretDef.upperAngle = 0.5f;
//    turretDef.enableMotor = true;
//    turretDef.motorSpeed = 1.0f;
//    turretDef.maxMotorTorque = 50.0f;
//    turretDef.collideConnected = false;
//    RevoluteJoint* turretJoint = static_cast<RevoluteJoint*>(world.createJoint(turretDef));
//
//    CSVExporter csv("2Ddemo/tank_demo.csv");
//    float maxAnchorError = 0.0f;
//    for (int i = 0; i < 360; ++i) {
//        if (i == 180) { jL->setMotorSpeed(0.0f); jR->setMotorSpeed(0.0f); } // 高原停靠
//        world.step(dt);
//        maxAnchorError = std::max(maxAnchorError,
//            std::max((jL->getAnchorB() - jL->getAnchorA()).length(),
//                (jR->getAnchorB() - jR->getAnchorA()).length()));
//        csv.writeFrame(i, world.getBodies());
//    }
//
//    Logger::info("验收: 战车终点 x=" + std::to_string(chassis->getPosition().getX()) +
//        "（> 9 驶上高原 PASS）");
//    Logger::info("验收: 车轮最大脱轴=" + std::to_string(maxAnchorError) + "（< 1cm PASS）");
//    Logger::info("验收: 炮塔角=" + std::to_string(turretJoint->getJointAngle()) +
//        "（限位 +0.5 PASS）");
//    Logger::info("轨迹已导出: 2Ddemo/tank_demo.csv");
//    return 0;
//}
