// GoogleTest 版 V4/003：滑块马达 + 摩擦关节测试
// 场景 1 电梯重载垂直匀速巡航：100kg 电梯箱 + 3 m/s 马达 + 2000N 推力上限，
//   克服 980N 自重爬升，速度严格收敛并维持在 3.000000 m/s
// 场景 2 斜面摩擦自锁：30° 光滑斜坡（无碰撞摩擦），FrictionJoint maxForce=100N
//   刹住 50N 下滑分力，300 帧零滑动漂移（对照组无关节自由下滑做对照）
// 场景 3 顶视自转摩擦制动：无重力圆盘 10 rad/s，角摩擦 2 N·m 线性衰减精确刹死，
//   无反转过冲
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "PrismaticJoint.h"
#include "FrictionJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>

// --- 场景 1: 电梯重载垂直匀速巡航测试 (Elevator Constant Speed) ---
TEST(V4_003, ElevatorConstantSpeed) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* rail = new Body(new Box(0.4f, 0.4f), 0.0f, 0.0f, 0.0f); // 静态轨道基座
    world.addBody(rail);

    // 100kg 重载电梯箱（自重 980N），挂在垂直导轨上
    Body* elevator = new Body(new Box(1.0f, 1.0f), 0.0f, 2.0f, 100.0f);
    world.addBody(elevator);

    PrismaticJointDef def;
    def.bodyA = rail;
    def.bodyB = elevator;
    def.localAnchorA = Vector2(0.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.localAxisA = Vector2(0.0f, 1.0f); // 垂直导轨
    def.referenceAngle = 0.0f;
    def.enableMotor = true;
    def.motorSpeed = 3.0f;       // 目标巡航速度 3 m/s
    def.maxMotorForce = 2000.0f; // 最大推力 2000N >> 自重 980N
    PrismaticJoint* joint = static_cast<PrismaticJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    float minVy = 1e9f, maxVy = -1e9f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        if (i >= 40) { // 跳过启动段（~18 帧内爬升到巡航速度）
            minVy = std::min(minVy, elevator->getVelocity().getY());
            maxVy = std::max(maxVy, elevator->getVelocity().getY());
        }
        if (std::isnan(elevator->getPosition().getY())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    // 巡航速度严格维持在 3.000000 m/s
    EXPECT_NEAR(elevator->getVelocity().getY(), 3.0f, 1e-4f);
    EXPECT_GT(minVy, 3.0f - 1e-4f);
    EXPECT_LT(maxVy, 3.0f + 1e-4f);
    // 稳态马达推力 = 对抗重力的 980N（100kg × 9.8），马达持续做功
    EXPECT_NEAR(joint->getMotorForce(dt), 980.0f, 5.0f);
    // 电梯确实在爬升（2m 起步 + ~14.5m），且全程不旋转
    EXPECT_GT(elevator->getPosition().getY(), 15.0f);
    EXPECT_LT(std::abs(elevator->getRotation()), 1e-4f);
}

// --- 场景 2: 斜面摩擦自锁测试 (Friction Joint Slope Holding) ---
// 30° 光滑斜坡（接触摩擦设 0），圆盘靠 FrictionJoint 刹住 50N 下滑分力。
// 注意：本引擎"先积分后求解"，纯耗散关节无法撤销已积分的本帧位移——
// 圆盘以 g∥·dt ≈ 0.0014 m/帧 蠕变，~30 帧后低速入睡冻结（总漂移 ~0.04m）。
// 对照组（无关节）自由下滑漂移 > 1m，证明刹车是摩擦关节在做功。
TEST(V4_003, FrictionJointSlopeHolding) {
    float dt = 1.0f / 60.0f;
    const Vector2 spawn(0.5f, 0.866f); // 圆盘贴在 30° 斜坡表面上的出生点

    // ===== 主场景：带摩擦关节 =====
    {
        World world(Vector2(0, -9.8f));

        // 30° 斜坡：Box(20,1) 绕质心转 -30°，顶面法线 n=(0.5, 0.866)
        Body* slope = new Body(new Box(20.0f, 1.0f), 0.0f, 0.0f, 0.0f);
        slope->setRotation(-0.5236f);
        world.addBody(slope);
        // 静态地面挂点（摩擦关节的 bodyA），放在斜坡上方不接触任何物体
        Body* ground = new Body(new Box(0.4f, 0.4f), 3.0f, 4.0f, 0.0f);
        world.addBody(ground);

        // 圆盘：半径 0.5，质量 10.2（自重 100N → 下滑分力 50N）
        float density = 10.2f / (Settings::PAI * 0.25f);
        Body* disk = new Body(new Circle(0.5f), spawn.getX(), spawn.getY(), density);
        disk->setFriction(0.0f); // 光滑斜坡：接触摩擦清零，刹车全靠摩擦关节
        world.addBody(disk);

        FrictionJointDef def;
        def.initialize(ground, disk, spawn); // 锚点 = 圆盘出生中心
        def.maxForce = 100.0f;  // 50N < 100N：刹得住
        def.maxTorque = 0.0f;
        FrictionJoint* joint = static_cast<FrictionJoint*>(world.createJoint(def));
        ASSERT_NE(joint, nullptr);

        for (int i = 0; i < 300; ++i) world.step(dt);

        // 速度被刹死、位移零滑动漂移（蠕变 + 睡眠冻结，见文件头注释）
        EXPECT_LT(disk->getVelocity().length(), 1e-3f);
        EXPECT_LT((disk->getPosition() - spawn).length(), 0.06f);
        EXPECT_FALSE(std::isnan(disk->getPosition().getX()));
    }

    // ===== 对照组：无摩擦关节，自由下滑 =====
    {
        World world(Vector2(0, -9.8f));

        Body* slope = new Body(new Box(20.0f, 1.0f), 0.0f, 0.0f, 0.0f);
        slope->setRotation(-0.5236f);
        world.addBody(slope);
        float density = 10.2f / (Settings::PAI * 0.25f);
        Body* disk = new Body(new Circle(0.5f), spawn.getX(), spawn.getY(), density);
        disk->setFriction(0.0f);
        world.addBody(disk);

        for (int i = 0; i < 300; ++i) world.step(dt);

        // 无关节：滑下斜坡（0.5·g·sin30°·t²）并飞出，漂移远超 1m
        EXPECT_GT((disk->getPosition() - spawn).length(), 1.0f);
    }
}

// --- 场景 3: 顶视自转摩擦制动测试 (Angular Friction Damping) ---
// 无重力平面，圆盘 10 rad/s 初始自转 + 角摩擦 maxTorque=2 N·m：
// 理论减速率 τ/I = 2/0.125 = 16 rad/s²，平滑线性衰减、精确刹死归零、
// 全程无反转过冲（限位冲量只对抗运动方向）
TEST(V4_003, AngularFrictionDamping) {
    World world(Vector2(0.0f, 0.0f)); // 顶视图：无重力
    float dt = 1.0f / 60.0f;

    Body* ground = new Body(new Box(0.4f, 0.4f), -1.5f, 0.0f, 0.0f); // 静态挂点
    world.addBody(ground);

    // 圆盘：半径 0.5，质量 1 → 转动惯量 I = m·r²/2 = 0.125
    float density = 1.0f / (Settings::PAI * 0.25f);
    Body* disk = new Body(new Circle(0.5f), 0.0f, 0.0f, density);
    world.addBody(disk);
    disk->setAngularVelocity(10.0f);

    FrictionJointDef def;
    def.initialize(ground, disk, Vector2(0.0f, 0.0f)); // 锚点 = 圆盘中心
    def.maxForce = 0.0f;
    def.maxTorque = 2.0f; // 最大角摩擦扭矩 2 N·m
    FrictionJoint* joint = static_cast<FrictionJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    float minOmega = 1e9f;
    bool stopped = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        float omega = disk->getAngularVelocity();
        minOmega = std::min(minOmega, omega);
        // 平滑线性衰减：每帧 -dt·τmax/I = -0.2667，帧 0 即开始衰减
        // → 帧 20 理论值 10 - 21×0.2667 ≈ 4.40
        if (i == 20) EXPECT_NEAR(omega, 4.4f, 0.05f);
        // 刹死后稳定停在 0，不再动弹（无振荡）
        if (!stopped && std::abs(omega) < 0.01f) stopped = true;
        if (stopped) EXPECT_LE(std::abs(omega), 0.01f);
    }

    EXPECT_GE(minOmega, -1e-6f);                              // 全程无反转过冲
    EXPECT_NEAR(disk->getAngularVelocity(), 0.0f, 1e-5f);     // 精确刹死归零
    EXPECT_FALSE(std::isnan(disk->getAngularVelocity()));
}
