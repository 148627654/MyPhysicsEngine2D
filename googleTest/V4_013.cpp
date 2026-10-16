// GoogleTest 版 V4/013：重装战车与破坏链锤（Day 13）
// 场景 1 刚性硬悬挂战车：梯形车身 + 双 WheelJoint 硬悬挂 + 轮毂马达，
//   20° 陡坡翻越驶上高原平台、炮塔限位旋转、车轮零脱轴
// 场景 2 破坏链锤与鼠标拖拽：RopeJoint(4m) 悬挂 50kg 铁球，MouseJoint 圆周
//   蓄力甩动（松弛段冲量严格为 0），释放后切线飞出砸穿 20 块砖墙，
//   绳索在极限拉直瞬间牢牢拽住链锤
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "WheelJoint.h"
#include "RevoluteJoint.h"
#include "RopeJoint.h"
#include "MouseJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Polygon.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>
#include <vector>

// --- 场景 1: 刚性硬悬挂战车 20° 陡坡翻越 + 炮塔限位 (Tank Slope Climb) ---
TEST(V4_013, TankSlopeClimb) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;
    const float slopeAngle = 20.0f * Settings::PAI / 180.0f; // 0.349 rad
    const float wheelRadius = 0.4f;

    // 20° 斜坡：Box(15.2, 0.5) 绕质心转 +20°，顶面直线 y = tan20°·x + 1.2959，
    // 末端 x ≈ 7.14（表面 3.90）。坡顶台阶必须无缝：若平台顶面高出坡面 0.5m，
    // 刚性悬挂的轮子压死在垂直断面上爬不上去（实测卡死）
    Body* slope = new Body(new Box(15.2f, 0.5f), 0.0f, 1.0371f, 0.0f);
    slope->setRotation(slopeAngle);
    slope->setFriction(0.9f);
    world.addBody(slope);
    // 高原平台：顶面 3.90 = 斜坡末端表面高度（5cm 微台阶可平滑滚过）
    Body* plateau = new Body(new Box(15.0f, 1.0f), 14.5f, 3.40f, 0.0f);
    plateau->setFriction(0.9f);
    world.addBody(plateau);

    // 梯形车身（底 3 顶 2 高 0.8，CCW），密度 2 → 质量 4，预俯仰 20° 出生即贴地
    Vector2 trapVerts[4] = {
        Vector2(-1.5f, -0.4f), Vector2(1.5f, -0.4f),
        Vector2(1.0f, 0.4f), Vector2(-1.0f, 0.4f)
    };
    Polygon* trap = new Polygon();
    ASSERT_TRUE(trap->set(trapVerts, 4));
    Body* chassis = new Body(trap, 0.0f, 2.2f, 2.0f);
    chassis->setRotation(slopeAngle);
    world.addBody(chassis);

    // 悬挂锚点（车身局部 (±1.2, -0.5) 旋转 20° 后）：两轮心连线精确平行坡面
    Vector2 anchorL = chassis->getPosition() + Vector2(-1.2f, -0.5f).rotate(slopeAngle);
    Vector2 anchorR = chassis->getPosition() + Vector2(1.2f, -0.5f).rotate(slopeAngle);

    Body* wheelL = new Body(new Circle(wheelRadius), anchorL.getX(), anchorL.getY() + 0.002f, 2.0f);
    Body* wheelR = new Body(new Circle(wheelRadius), anchorR.getX(), anchorR.getY() + 0.002f, 2.0f);
    wheelL->setFriction(0.9f);
    wheelR->setFriction(0.9f);
    world.addBody(wheelL);
    world.addBody(wheelR);

    WheelJointDef defL;
    defL.Initialize(chassis, wheelL, anchorL, Vector2(0.0f, -1.0f));
    defL.enableMotor = true;
    defL.motorSpeed = -12.0f; // 顺时针 → 向右（上坡）滚进
    defL.maxMotorTorque = 50.0f;
    defL.collideConnected = false;
    WheelJoint* jL = static_cast<WheelJoint*>(world.createJoint(defL));
    ASSERT_NE(jL, nullptr);

    WheelJointDef defR;
    defR.Initialize(chassis, wheelR, anchorR, Vector2(0.0f, -1.0f));
    defR.enableMotor = true;
    defR.motorSpeed = -12.0f;
    defR.maxMotorTorque = 50.0f;
    defR.collideConnected = false;
    WheelJoint* jR = static_cast<WheelJoint*>(world.createJoint(defR));
    ASSERT_NE(jR, nullptr);

    // 旋转炮塔：车顶 Revolute 铰接 + 角度限位 [-0.5, +0.5] + 马达驱动
    Body* turret = new Body(new Box(1.0f, 0.3f), 0.0f, 2.65f, 2.0f);
    world.addBody(turret);
    RevoluteJointDef turretDef;
    turretDef.bodyA = chassis;
    turretDef.bodyB = turret;
    turretDef.localAnchorA = Vector2(0.0f, 0.45f); // 车顶
    turretDef.localAnchorB = Vector2(0.0f, 0.0f);
    turretDef.enableLimit = true;
    turretDef.lowerAngle = -0.5f;
    turretDef.upperAngle = 0.5f;
    turretDef.enableMotor = true;
    turretDef.motorSpeed = 1.0f;
    turretDef.maxMotorTorque = 50.0f;
    turretDef.collideConnected = false;
    RevoluteJoint* turretJoint = static_cast<RevoluteJoint*>(world.createJoint(turretDef));
    ASSERT_NE(turretJoint, nullptr);

    // 20° 陡坡翻越（~120 帧驶上高原），随后马达断电在坡顶自然停靠
    float maxAnchorError = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        if (i == 160) { // 已在高原平台上：断电停靠
            jL->setMotorSpeed(0.0f);
            jR->setMotorSpeed(0.0f);
        }
        world.step(dt);
        maxAnchorError = std::max(maxAnchorError,
            std::max((jL->getAnchorB() - jL->getAnchorA()).length(),
                (jR->getAnchorB() - jR->getAnchorA()).length()));
        if (std::isnan(chassis->getPosition().getX()) ||
            std::isnan(wheelL->getPosition().getY())) { hasNaN = true; break; }
    }

    Vector2 pos = chassis->getPosition();
    // 战车成功驶上 20° 斜坡并停在坡顶高原（x > 9 进入平台区，车身回平）
    EXPECT_GT(pos.getX(), 9.0f);
    EXPECT_LT(std::abs(chassis->getRotation()), 0.1f); // 高原上回正
    // 车轮零脱轴：全程悬挂锚点误差 < 1cm（刚性硬悬挂）
    EXPECT_LT(maxAnchorError, 0.01f);
    // 炮塔旋转并停在限位 +0.5（马达驱动 + 角度限位）
    EXPECT_NEAR(turretJoint->getJointAngle(), 0.5f, 0.05f);
    EXPECT_FALSE(hasNaN);
}

// --- 场景 2: 破坏链锤与鼠标交互拖拽 (Flail Swing Release) ---
// 悬挂点 (0,8) + 50kg 铁球 + RopeJoint(4m)。MouseJoint 拖动目标点绕
// (0,6.5) 半径 2 圆周蓄力（球距锚点 < 4m：松弛段冲量严格为 0），在最低点
// 释放 → 球沿切线向右飞出（~4 m/s）→ 砸穿 20 块砖墙 → 绳索在距锚 4m 处
// 极限拉直牢牢拽住链锤
TEST(V4_013, FlailSwingRelease) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // 悬挂点（静态）
    Body* anchor = new Body(new Box(0.4f, 0.4f), 0.0f, 8.0f, 0.0f);
    world.addBody(anchor);
    // 50kg 铁球，出生在 (0, 6.5)（距锚 1.5m，绳松弛）
    float ballDensity = 50.0f / (Settings::PAI * 0.25f);
    Body* ball = new Body(new Circle(0.5f), 0.0f, 6.5f, ballDensity);
    world.addBody(ball);

    RopeJointDef ropeDef;
    ropeDef.bodyA = anchor;
    ropeDef.bodyB = ball;
    ropeDef.localAnchorA = Vector2(0.0f, 0.0f);
    ropeDef.localAnchorB = Vector2(0.0f, 0.0f);
    ropeDef.maxLength = 4.0f;
    ropeDef.collideConnected = false;
    RopeJoint* rope = static_cast<RopeJoint*>(world.createJoint(ropeDef));
    ASSERT_NE(rope, nullptr);

    // 鼠标关节：抓取铁球（bodyA 用远处静态地基）
    Body* ground = new Body(new Box(0.4f, 0.4f), -10.0f, -10.0f, 0.0f);
    world.addBody(ground);
    MouseJointDef mouseDef;
    mouseDef.bodyA = ground;
    mouseDef.bodyB = ball;
    mouseDef.target = Vector2(0.0f, 6.5f);
    mouseDef.localAnchorB = Vector2(0.0f, 0.0f);
    mouseDef.maxForce = 20000.0f;
    mouseDef.frequencyHz = 5.0f;
    mouseDef.dampingRatio = 0.7f;
    mouseDef.collideConnected = false;
    MouseJoint* mouse = static_cast<MouseJoint*>(world.createJoint(mouseDef));
    ASSERT_NE(mouse, nullptr);

    // 20 块砖墙（4 列 × 5 层，0.25²，密度 10 → 0.625kg/块）
    std::vector<Body*> bricks;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 5; ++r) {
            bricks.push_back(world.createBox(0.25f, 0.25f,
                1.725f + c * 0.25f, 4.125f + r * 0.25f, 10.0f));
        }
    }

    // 阶段 1：圆周蓄力甩动 150 帧（目标点绕 (0,6.5) 半径 2、角速度 2 rad/s）
    for (int i = 0; i < 150; ++i) {
        float theta = 2.0f * (i + 1) * dt; // 从 0 起转
        mouse->setTarget(Vector2(2.0f * std::cos(theta), 6.5f + 2.0f * std::sin(theta)));
        world.step(dt);
    }
    // 松弛段冲量严格为 0：全程球距锚点 < 4m，绳索从未绷紧
    EXPECT_EQ(rope->getImpulse(), 0.0f);

    // 阶段 2：释放（DestroyJoint 抛射）→ 切线飞出砸墙 → 绳索极限拉直拽住
    world.destroyJoint(mouse);

    float maxRopeLen = 0.0f;
    float minRopeImpulse = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 240; ++i) {
        world.step(dt);
        maxRopeLen = std::max(maxRopeLen, (ball->getPosition() - anchor->getPosition()).length());
        minRopeImpulse = std::min(minRopeImpulse, rope->getImpulse());
        if (std::isnan(ball->getPosition().getX())) { hasNaN = true; break; }
    }

    // 绳索在极限拉直瞬间牢牢拽住链锤：全程球距锚点 ≤ 4m + 微小修正余量
    EXPECT_LE(maxRopeLen, 4.05f);
    // 绷紧瞬间产生强大拉力（λ < 0 约定）
    EXPECT_LT(minRopeImpulse, -1.0f);
    // 碎砖横飞：至少 5 块砖被撞飞 0.8m 以上（索引反推出生位置：idx = c·5 + r）
    int scattered = 0;
    for (int idx = 0; idx < (int)bricks.size(); ++idx) {
        int c = idx / 5, r = idx % 5;
        Vector2 spawn(1.725f + c * 0.25f, 4.125f + r * 0.25f);
        if ((bricks[idx]->getPosition() - spawn).length() > 0.8f) ++scattered;
    }
    EXPECT_GE(scattered, 5);
    EXPECT_FALSE(hasNaN);
}
