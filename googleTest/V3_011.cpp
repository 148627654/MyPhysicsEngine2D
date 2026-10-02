// GoogleTest 版 V3/011：RevoluteJoint 角度限位 + 马达测试
// 移植自 tests/V3/011.cpp，场景逻辑原样保留，断言改为 gtest 宏
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RevoluteJoint.h"
#include "Box.h"
#include "Logger.h"
#include <cmath>

// 静态销钉（触发器：纯锚点载体，避免与挂载物物理重叠导致睡眠冻结）
static Body* MakePin(float x, float y) {
    Body* pin = new Body(new Box(1.0f, 1.0f), x, y, 0.0f);
    pin->getShape()->isTrigger = true;
    return pin;
}

// --- 场景 1: 马达驱动恒速自转测试 (Motor Drive Test) ---
// 轮子铰接在静态车身上，motorSpeed=10 rad/s、maxTorque=100 N·m，120 帧后应稳定巡航
TEST(V3_011, MotorDriveConstantSpeed) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* chassis = MakePin(0.0f, 5.0f); // 静态车身（锚点载体）
    world.addBody(chassis);
    Body* wheel = new Body(new Box(1.0f, 1.0f), 0.0f, 5.0f, 1.0f); // 轮子
    wheel->setSleepAllow(false);
    world.addBody(wheel);

    RevoluteJointDef def;
    def.initialize(chassis, wheel, Vector2(0.0f, 5.0f));
    def.enableMotor = true;
    def.motorSpeed = 10.0f;     // 目标转速 10 rad/s
    def.maxMotorTorque = 100.0f;// 最大扭矩 100 N·m
    RevoluteJoint* joint = static_cast<RevoluteJoint*>(world.createJoint(def));

    for (int i = 0; i < 120; ++i) world.step(dt);

    float w = wheel->getAngularVelocity();
    float torque = joint->getMotorTorque(dt);
    // 转速迅速稳定在 10 rad/s；无负载稳态下扭矩收敛（不超过最大扭矩）
    EXPECT_NEAR(w, 10.0f, 0.1f);
    EXPECT_LE(std::abs(torque), 100.0f + 1e-3f);
}

// --- 场景 2: 摆臂硬边界限位测试 (Angle Limits Rebound Test) ---
// 摆臂装配在 90°（π/2），限位 [-π/4, +π/4]（相对装配角），
// 从 90° 释放下坠，冲击 +45°（绝对角）边界时应被硬生生挡住并反弹
TEST(V3_011, AngleLimitsRebound) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* pin = MakePin(0.0f, 5.0f);
    world.addBody(pin);

    // 摆臂：长条 4×0.5，装配在 90°（锚点在其一端 (0,5)，质心在 (0,7)）
    Body* arm = new Body(new Box(4.0f, 0.5f), 0.0f, 7.0f, 1.0f);
    arm->setRotation(Settings::PAI / 2.0f);
    // 竖直倒立时质心在锚点正上方，重力矩为零（不稳定平衡，数值上不会倒下），
    // 给初始角速度模拟真实世界的微扰；禁止入睡（下落加速期速度低于睡眠阈值，
    // 0.5 秒就会被冻结在半空中）
    arm->setAngularVelocity(-1.0f); // 负方向微扰：摆臂向下坠，冲击 -45°（绝对角 +45°）限位
    arm->setSleepAllow(false);
    world.addBody(arm);

    RevoluteJointDef def;
    def.initialize(pin, arm, Vector2(0.0f, 5.0f));
    def.enableLimit = true;
    def.lowerAngle = -Settings::PAI / 4.0f;
    def.upperAngle = Settings::PAI / 4.0f;
    RevoluteJoint* joint = static_cast<RevoluteJoint*>(world.createJoint(def));

    float minAngle = 1e9f, maxAngle = -1e9f;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        float a = joint->getJointAngle();
        minAngle = std::min(minAngle, a);
        maxAngle = std::max(maxAngle, a);
    }

    float lo = -Settings::PAI / 4.0f;
    float hi = Settings::PAI / 4.0f;
    // 相对角度全程受控在 [-45°, +45°] 内（最大超界 < 0.005 rad），且确实冲击到了下限位
    EXPECT_GE(minAngle, lo - 0.005f);
    EXPECT_LE(maxAngle, hi + 0.005f);
    EXPECT_LT(minAngle, lo + 0.01f);
}

// --- 场景 3: 等角锁死刚性焊接测试 (Equal Limits Lock Test) ---
// lower = upper = 0：销钉退化为刚性焊接关节，巨大扭矩也拧不动
TEST(V3_011, EqualLimitWeld) {
    World world(Vector2(0, 0)); // 零重力：纯扭矩对抗
    float dt = 1.0f / 60.0f;

    Body* pin = MakePin(0.0f, 5.0f);
    world.addBody(pin);
    Body* plate = new Body(new Box(2.0f, 0.5f), 0.0f, 5.0f, 1.0f);
    plate->setSleepAllow(false);
    world.addBody(plate);

    RevoluteJointDef def;
    def.initialize(pin, plate, Vector2(0.0f, 5.0f));
    def.enableLimit = true;
    def.lowerAngle = 0.0f;
    def.upperAngle = 0.0f; // 等角锁死 → 刚性焊接
    RevoluteJoint* joint = static_cast<RevoluteJoint*>(world.createJoint(def));

    float maxAbsAngle = 0.0f;
    for (int i = 0; i < 120; ++i) {
        plate->addTorque(50.0f); // 每帧施加巨大扭矩试图拧动（Δω ≈ 2.4 rad/s/帧）
        world.step(dt);
        maxAbsAngle = std::max(maxAbsAngle, std::abs(joint->getJointAngle()));
    }

    // 相对角度完全锁死为 0（刚性焊接）
    EXPECT_LT(maxAbsAngle, 0.01f);
}
