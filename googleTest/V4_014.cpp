// GoogleTest 版 V4/014：全量回归与压力测试汇总（Day 14）
// 场景 1 关节体系大一统：11 种关节（含齿轮的 2 个父关节）全部经工厂创建并共存
//   求解，零崩溃零 NaN——V4 完整 API 表面的终极烟雾测试
// 场景 2 布娃娃零退化回归：6 刚体 + 5 限位铰链的布娃娃摔台阶，V4 引擎下
//   关节咬合、无爆炸无 NaN
// 场景 3 双摆混沌零退化回归：600 帧混沌轨迹有界、无 NaN
// 场景 4 闭环四连杆零退化回归：两端销接的闭环链 1000 帧，锚点漂移 < 1e-3
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "DistanceJoint.h"
#include "SpringJoint.h"
#include "RevoluteJoint.h"
#include "WeldJoint.h"
#include "PrismaticJoint.h"
#include "FrictionJoint.h"
#include "RopeJoint.h"
#include "PulleyJoint.h"
#include "GearJoint.h"
#include "WheelJoint.h"
#include "MouseJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>
#include <vector>

// --- 场景 1: 11 种关节工厂大合体烟雾测试 (All Joints Factory Smoke) ---
TEST(V4_014, AllJointsFactorySmoke) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // 静态地基 + 12 个动态刚体（横向排开互不接触）
    Body* ground = new Body(new Box(0.4f, 0.4f), 0.0f, 8.0f, 0.0f);
    world.addBody(ground);
    std::vector<Body*> bodies;
    for (int i = 0; i < 12; ++i) {
        bodies.push_back(world.createBox(0.5f, 0.5f, -8.0f + i * 1.6f, 4.0f, 1.0f));
    }

    // 1. Distance 刚性杆
    DistanceJointDef dDef;
    dDef.bodyA = ground; dDef.bodyB = bodies[0];
    dDef.localAnchorA = Vector2(0.0f, -4.0f);
    dDef.localAnchorB = Vector2(0.0f, 0.0f);
    world.createJoint(dDef);

    // 2. Spring 软弹簧
    SpringJointDef sDef;
    sDef.bodyA = ground; sDef.bodyB = bodies[1];
    sDef.localAnchorA = Vector2(0.0f, -4.0f);
    sDef.localAnchorB = Vector2(0.0f, 0.0f);
    sDef.length = 4.0f;
    world.createJoint(sDef);

    // 3. Revolute 铰链
    RevoluteJointDef rDef;
    rDef.bodyA = ground; rDef.bodyB = bodies[2];
    rDef.localAnchorA = Vector2(0.0f, -4.0f);
    rDef.localAnchorB = Vector2(0.0f, 0.0f);
    world.createJoint(rDef);

    // 4. Weld 焊接
    WeldJointDef wDef;
    wDef.bodyA = ground; wDef.bodyB = bodies[3];
    wDef.localAnchorA = Vector2(0.0f, -4.0f);
    wDef.localAnchorB = Vector2(0.0f, 0.0f);
    world.createJoint(wDef);

    // 5. Prismatic 滑块
    PrismaticJointDef pDef;
    pDef.bodyA = ground; pDef.bodyB = bodies[4];
    pDef.localAnchorA = Vector2(0.0f, -4.0f);
    pDef.localAnchorB = Vector2(0.0f, 0.0f);
    pDef.localAxisA = Vector2(0.0f, 1.0f);
    world.createJoint(pDef);

    // 6. Friction 摩擦
    FrictionJointDef fDef;
    fDef.bodyA = ground; fDef.bodyB = bodies[5];
    fDef.localAnchorA = Vector2(0.0f, -4.0f);
    fDef.localAnchorB = Vector2(0.0f, 0.0f);
    fDef.maxForce = 100.0f;
    fDef.maxTorque = 50.0f;
    world.createJoint(fDef);

    // 7. Rope 绳
    RopeJointDef ropeDef;
    ropeDef.bodyA = ground; ropeDef.bodyB = bodies[6];
    ropeDef.localAnchorA = Vector2(0.0f, -4.0f);
    ropeDef.localAnchorB = Vector2(0.0f, 0.0f);
    ropeDef.maxLength = 4.0f;
    world.createJoint(ropeDef);

    // 8. Pulley 滑轮（两刚体 + 地面锚点）
    PulleyJointDef pulDef;
    pulDef.bodyA = bodies[7]; pulDef.bodyB = bodies[8];
    pulDef.groundAnchorA = Vector2(-8.0f, 8.0f);
    pulDef.groundAnchorB = Vector2(-6.4f, 8.0f);
    pulDef.localAnchorA = Vector2(0.0f, 0.0f);
    pulDef.localAnchorB = Vector2(0.0f, 0.0f);
    pulDef.ratio = 1.0f;
    pulDef.collideConnected = false;
    world.createJoint(pulDef);

    // 9. Gear 齿轮（需要 2 个父关节：两个 Revolute）
    RevoluteJointDef g1Def;
    g1Def.bodyA = ground; g1Def.bodyB = bodies[9];
    g1Def.localAnchorA = Vector2(0.0f, -4.0f);
    g1Def.localAnchorB = Vector2(0.0f, 0.0f);
    RevoluteJoint* g1 = static_cast<RevoluteJoint*>(world.createJoint(g1Def));
    RevoluteJointDef g2Def;
    g2Def.bodyA = ground; g2Def.bodyB = bodies[10];
    g2Def.localAnchorA = Vector2(0.0f, -4.0f);
    g2Def.localAnchorB = Vector2(0.0f, 0.0f);
    RevoluteJoint* g2 = static_cast<RevoluteJoint*>(world.createJoint(g2Def));
    GearJointDef gearDef;
    gearDef.joint1 = g1; gearDef.joint2 = g2; gearDef.ratio = 1.0f;
    world.createJoint(gearDef);

    // 10. Wheel 车轮
    WheelJointDef wheelDef;
    wheelDef.bodyA = ground; wheelDef.bodyB = bodies[11];
    wheelDef.localAnchorA = Vector2(0.0f, -4.0f);
    wheelDef.localAnchorB = Vector2(0.0f, 0.0f);
    wheelDef.localAxisA = Vector2(0.0f, -1.0f);
    world.createJoint(wheelDef);

    // 11. Mouse 鼠标（地基 + 第 0 号刚体）
    MouseJointDef mouseDef;
    mouseDef.bodyA = ground; mouseDef.bodyB = bodies[0];
    mouseDef.target = bodies[0]->getPosition();
    mouseDef.localAnchorB = Vector2(0.0f, 0.0f);
    mouseDef.maxForce = 1000.0f;
    world.createJoint(mouseDef);

    // 关节总数：11 类 + 齿轮的 2 个父销钉 = 13
    EXPECT_EQ(world.getJointCount(), 13);

    // 全部关节共存求解 30 帧：零崩溃零 NaN
    bool hasNaN = false;
    for (int i = 0; i < 30; ++i) {
        world.step(dt);
        for (Body* b : bodies) {
            if (std::isnan(b->getPosition().getX()) ||
                std::isnan(b->getVelocity().getY())) { hasNaN = true; break; }
        }
        if (hasNaN) break;
    }
    EXPECT_FALSE(hasNaN);
}

// --- 场景 2: 布娃娃零退化回归 (Ragdoll Regression) ---
TEST(V4_014, RagdollRegression) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // 台阶（3 级，每级 1×0.3）
    for (int i = 0; i < 3; ++i)
        world.addBody(new Body(new Box(1.0f + i, 0.3f), 1.5f + i * 0.5f, 0.15f + i * 0.3f, 0.0f));

    // 布娃娃：躯干 + 头 + 双臂 + 双腿（6 刚体 5 限位铰链）
    Body* torso = world.createBox(1.0f, 1.5f, 2.0f, 6.0f, 2.0f);
    Body* head = world.createCircle(0.35f, 2.0f, 7.3f, 2.0f);
    Body* armL = world.createBox(0.2f, 0.8f, 1.25f, 6.4f, 2.0f);
    Body* armR = world.createBox(0.2f, 0.8f, 2.75f, 6.4f, 2.0f);
    Body* legL = world.createBox(0.25f, 1.0f, 1.65f, 4.75f, 2.0f);
    Body* legR = world.createBox(0.25f, 1.0f, 2.35f, 4.75f, 2.0f);

    auto addLimitedHinge = [&](Body* a, Body* b, const Vector2& anchor, bool collide) {
        RevoluteJointDef def;
        def.bodyA = a; def.bodyB = b;
        def.localAnchorA = (anchor - a->getPosition()).rotate(-a->getRotation());
        def.localAnchorB = (anchor - b->getPosition()).rotate(-b->getRotation());
        def.enableLimit = true;
        def.lowerAngle = -0.6f; def.upperAngle = 0.6f;
        def.collideConnected = collide;
        world.createJoint(def);
    };
    addLimitedHinge(torso, head, Vector2(2.0f, 6.9f), false);  // 颈
    addLimitedHinge(torso, armL, Vector2(1.4f, 6.5f), false);  // 左肩
    addLimitedHinge(torso, armR, Vector2(2.6f, 6.5f), false);  // 右肩
    addLimitedHinge(torso, legL, Vector2(1.75f, 5.2f), false); // 左髋
    addLimitedHinge(torso, legR, Vector2(2.25f, 5.2f), false); // 右髋

    float maxSpeed = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        maxSpeed = std::max(maxSpeed, torso->getVelocity().length());
        if (std::isnan(torso->getPosition().getX()) ||
            std::isnan(head->getPosition().getY())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxSpeed, 25.0f); // 无爆炸（自由落体峰值 ~8 m/s）
    // 关节咬合：头与躯干仍相连（颈铰链未断裂）
    EXPECT_LT((head->getPosition() - torso->getPosition()).length(), 1.3f);
}

// --- 场景 3: 双摆混沌零退化回归 (Double Pendulum Regression) ---
TEST(V4_014, DoublePendulumRegression) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* pin = new Body(new Box(0.4f, 0.4f), 0.0f, 5.0f, 0.0f);
    world.addBody(pin);
    Body* arm1 = world.createBox(0.2f, 2.0f, 0.0f, 4.0f, 2.0f);
    Body* arm2 = world.createBox(0.2f, 2.0f, 0.0f, 2.0f, 2.0f);

    RevoluteJointDef j1;
    j1.bodyA = pin; j1.bodyB = arm1;
    j1.localAnchorA = Vector2(0.0f, 0.0f);
    j1.localAnchorB = Vector2(0.0f, 1.0f);
    j1.collideConnected = false;
    world.createJoint(j1);

    RevoluteJointDef j2;
    j2.bodyA = arm1; j2.bodyB = arm2;
    j2.localAnchorA = Vector2(0.0f, -1.0f);
    j2.localAnchorB = Vector2(0.0f, 1.0f);
    j2.collideConnected = false;
    world.createJoint(j2);

    arm2->setAngularVelocity(5.0f); // 混沌初速微扰

    float maxReach = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 600; ++i) {
        world.step(dt);
        maxReach = std::max(maxReach, (arm2->getPosition() - pin->getPosition()).length());
        if (std::isnan(arm2->getPosition().getX()) ||
            std::isnan(arm2->getAngularVelocity())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxReach, 4.2f); // 混沌轨迹有界（两臂总长 4m + 微余量）
}

// --- 场景 4: 闭环四连杆零退化回归 (Closed Loop Linkage Regression) ---
// 闭环链：链长 3.1 + 2.1 + 3.1 = 8.3 > 销距 6（略长 → 自然悬垂成 V 形）。
// 若链长恰等于销距则静不定死锁（刚性杆无法下垂，四铰链修正互相打架残差 15mm）
TEST(V4_014, ClosedLoopRegression) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // 两端销钉（静态），间距 6
    Body* pin0 = new Body(new Box(0.4f, 0.4f), 0.0f, 0.0f, 0.0f);
    Body* pin1 = new Body(new Box(0.4f, 0.4f), 6.0f, 0.0f, 0.0f);
    world.addBody(pin0);
    world.addBody(pin1);

    // 悬垂 V 形装配（解析解）：M1=(1.95,-2.41) M2=(4.05,-2.41)
    const Vector2 M1(1.95f, -2.41f), M2(4.05f, -2.41f);
    Body* L1 = new Body(new Box(3.1f, 0.2f), (M1.getX()) * 0.5f, M1.getY() * 0.5f, 1.0f);
    L1->setRotation(std::atan2(M1.getY(), M1.getX()));
    world.addBody(L1);
    Body* L2 = new Body(new Box(2.1f, 0.2f), 3.0f, -2.41f, 1.0f);
    world.addBody(L2);
    Body* L3 = new Body(new Box(3.1f, 0.2f), 6.0f - (6.0f - M2.getX()) * 0.5f, M2.getY() * 0.5f, 1.0f);
    L3->setRotation(std::atan2(-M2.getY(), 6.0f - M2.getX()));
    world.addBody(L3);

    auto pinLink = [&](Body* a, Body* b, const Vector2& anchor) {
        RevoluteJointDef def;
        def.bodyA = a; def.bodyB = b;
        def.localAnchorA = (anchor - a->getPosition()).rotate(-a->getRotation());
        def.localAnchorB = (anchor - b->getPosition()).rotate(-b->getRotation());
        def.collideConnected = false;
        return static_cast<RevoluteJoint*>(world.createJoint(def));
    };
    RevoluteJoint* c0 = pinLink(pin0, L1, Vector2(0.0f, 0.0f));
    RevoluteJoint* c1 = pinLink(L1, L2, M1);
    RevoluteJoint* c2 = pinLink(L2, L3, M2);
    RevoluteJoint* c3 = pinLink(L3, pin1, Vector2(6.0f, 0.0f));

    // 1000 帧闭环漂移监测：四个销钉锚点误差 < 1e-3
    float maxAnchorError = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 1000; ++i) {
        world.step(dt);
        RevoluteJoint* joints[4] = { c0, c1, c2, c3 };
        for (RevoluteJoint* j : joints) {
            maxAnchorError = std::max(maxAnchorError,
                (j->getAnchorB() - j->getAnchorA()).length());
        }
        if (std::isnan(L2->getPosition().getY()) ||
            std::isnan(L2->getAngularVelocity())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxAnchorError, 1e-3f); // 闭环漂移 < 1e-3（V3 基准 0.000136）
}
