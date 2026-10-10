// GoogleTest 版 V4/007：WheelJoint 车轮关节测试
// 场景 1 轮轴绝对刚性锁死：车身固定悬空，剧烈外力（交替轴向推拉 + 轮缘大扭矩
//   + 对角斜拉）推拉车轮 300 帧，轮心相对悬挂点位移误差全程 < 1e-4 米（绝对不脱轴）
// 场景 2 悬空车轮马达恒速巡航：动态车身悬空，马达驱动轮毂目标转速 25 rad/s，
//   相对角速度 2 帧内收敛并精确维持 25.000000 rad/s，车身承受等大反向反作用
//   冲量反向自转（急加速"抬头"力学）
// 场景 3 15° 陡坡战车攀爬：梯形车身 + 2 个 WheelJoint 刚性硬悬挂 + 双轮马达，
//   战车克服重力下滑分力平稳爬坡，车轮不脱轴、车体无微震发散
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "WheelJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Polygon.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>

// --- 场景 1: 轮轴绝对刚性锁死测试 (Rigid Axle Lock) ---
TEST(V4_007, RigidAxleLock) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 车身固定在空中（静态底盘），悬挂点 (0, 2.5)
    Body* chassis = new Body(new Box(2.0f, 0.5f), 0.0f, 3.0f, 0.0f);
    world.addBody(chassis);

    // 车轮：半径 0.4、密度 2 → 质量 ≈ 1.005，出生在悬挂点正下方 (0, 2.5)
    Body* wheel = new Body(new Circle(0.4f), 0.0f, 2.5f, 2.0f);
    world.addBody(wheel);

    WheelJointDef def;
    def.bodyA = chassis;
    def.bodyB = wheel;
    def.localAnchorA = Vector2(0.0f, -0.5f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.localAxisA = Vector2(0.0f, -1.0f); // 悬挂支柱垂直向下
    def.collideConnected = false;
    WheelJoint* joint = static_cast<WheelJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    // 剧烈外力：交替轴向 ±500N 推拉 + 周期轮缘推挤（力臂 0.4 → 扭矩 120 N·m
    // 疯狂拨转车轮）+ 对角斜拉。轮心锚点 rB=0：无论车轮被拨多快，锚点几何不受影响
    float maxAnchorError = 0.0f;
    for (int i = 0; i < 300; ++i) {
        float fx = 0.0f, fy = 0.0f;
        switch (i % 4) {
        case 0: fx = 500.0f; break;
        case 1: fy = 500.0f; break;
        case 2: fx = -500.0f; break;
        default: fy = -500.0f; break;
        }
        wheel->addForce(Vector2(fx, fy));
        if (i % 5 == 0)
            wheel->applyForceAtPoint(Vector2(300.0f, 0.0f), wheel->getPosition() + Vector2(0.4f, 0.0f));
        if (i % 9 == 0)
            wheel->addForce(Vector2(350.0f, 350.0f));

        world.step(dt);
        Vector2 err = joint->getAnchorB() - joint->getAnchorA();
        maxAnchorError = std::max(maxAnchorError, err.length());
    }

    EXPECT_LT(maxAnchorError, 1e-4f); // 全程相对位移误差 < 1e-4 米（绝对不脱轴）
    EXPECT_FALSE(std::isnan(wheel->getPosition().getX()));
    EXPECT_FALSE(std::isnan(wheel->getAngularVelocity()));
}

// --- 场景 2: 悬空车轮马达恒速巡航测试 (Motor Speed Tracking) ---
// 悬空动态车身 + 轮毂马达 motorSpeed=25 rad/s：角有效质量线性精确解，无扭矩
// 饱和时首帧即可收敛到目标相对转速。车身承受等大反向反作用冲量 → 角动量
// 守恒下车身反向自转（wA ≈ -25·IB/(IA+IB) ≈ -0.84），车轮随之绕悬挂点公转
TEST(V4_007, MotorSpeedTracking) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 车身：3×0.5 箱、密度 2 → 质量 3、惯量 2.3125，悬空 (0, 10)
    Body* chassis = new Body(new Box(3.0f, 0.5f), 0.0f, 10.0f, 2.0f);
    world.addBody(chassis);
    // 车轮：半径 0.4、密度 2 → 质量 ≈ 1.005，悬挂点 (0, 9.5)
    Body* wheel = new Body(new Circle(0.4f), 0.0f, 9.5f, 2.0f);
    world.addBody(wheel);

    WheelJointDef def;
    def.bodyA = chassis;
    def.bodyB = wheel;
    def.localAnchorA = Vector2(0.0f, -0.5f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.enableMotor = true;
    def.motorSpeed = 25.0f;      // 目标驱动转速 25 rad/s
    def.maxMotorTorque = 100.0f; // 扭矩上限（首帧略受钳位，第 2 帧即达目标）
    def.collideConnected = false;
    WheelJoint* joint = static_cast<WheelJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    float maxSpeedError = 0.0f;
    for (int i = 0; i < 120; ++i) {
        world.step(dt);
        float relW = wheel->getAngularVelocity() - chassis->getAngularVelocity();
        if (i >= 10) // 跳过启动段（实际 2 帧内收敛，余量留足）
            maxSpeedError = std::max(maxSpeedError, std::abs(relW - 25.0f));
    }

    float relW = wheel->getAngularVelocity() - chassis->getAngularVelocity();
    EXPECT_NEAR(relW, 25.0f, 1e-4f); // 相对角速度稳定维持在 25.000000 rad/s
    EXPECT_LT(maxSpeedError, 1e-4f); // 收敛后全程零漂移
    // 反作用力学：车身被马达反作用扭矩驱动反向自转（角动量守恒，wA < 0）
    EXPECT_LT(chassis->getAngularVelocity(), -0.1f);
    // 车身自转下车轮绕悬挂点公转，轮轴依然绝对锁死（轨道误差 < 1e-4）
    EXPECT_LT((joint->getAnchorB() - joint->getAnchorA()).length(), 1e-4f);
    EXPECT_FALSE(std::isnan(chassis->getPosition().getY()));
}

// --- 场景 3: 15° 陡坡战车攀爬测试 (Steep Slope Climbing) ---
// 梯形车身（底 3 顶 2 高 0.8）+ 2 个 WheelJoint 刚性硬悬挂 + 双轮马达驱动。
// 车身与车轮预装配成 15° 俯仰（悬挂锚点连线精确平行于坡面，两轮出生即贴地）。
// 马达驱动车轮以相对 -12 rad/s 恒速自转（轮半径 0.4 → 无滑动巡航 4.8 m/s），
// 战车克服重力下滑分力平稳爬坡
TEST(V4_007, SteepSlopeClimbing) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;
    const float slopeAngle = 15.0f * Settings::PAI / 180.0f; // 0.2618 rad
    const float wheelRadius = 0.4f;

    // 15° 斜坡：Box(80, 0.5) 绕质心转 +15°，顶面直线 y = tan15°·x + 1.2959
    Body* slope = new Body(new Box(80.0f, 0.5f), 0.0f, 1.0371f, 0.0f);
    slope->setRotation(slopeAngle);
    slope->setFriction(0.9f); // 轮胎抓地力来源
    world.addBody(slope);

    // 梯形车身：底 3.0、顶 2.0、高 0.8（CCW），密度 2 → 质量 4，
    // 质心 (0, 2.2)、俯仰 15°
    Vector2 trapVerts[4] = {
        Vector2(-1.5f, -0.4f), Vector2(1.5f, -0.4f),
        Vector2(1.0f, 0.4f), Vector2(-1.0f, 0.4f)
    };
    Polygon* trap = new Polygon();
    ASSERT_TRUE(trap->set(trapVerts, 4));
    Body* chassis = new Body(trap, 0.0f, 2.2f, 2.0f);
    chassis->setRotation(slopeAngle);
    world.addBody(chassis);

    // 悬挂锚点：车身局部 (±1.2, -0.5) 旋转 15° 后的世界坐标
    // → 左轮 (-1.0297, 1.4064)、右轮 (1.2885, 2.0276)，两轮心连线精确平行坡面
    Vector2 anchorL = chassis->getPosition() + Vector2(-1.2f, -0.5f).rotate(slopeAngle);
    Vector2 anchorR = chassis->getPosition() + Vector2(1.2f, -0.5f).rotate(slopeAngle);

    // 双轮：半径 0.4、密度 2 → 质量 ≈ 1.005，各留 2mm 缝隙重力落位
    Body* wheelL = new Body(new Circle(wheelRadius), anchorL.getX(), anchorL.getY() + 0.002f, 2.0f);
    Body* wheelR = new Body(new Circle(wheelRadius), anchorR.getX(), anchorR.getY() + 0.002f, 2.0f);
    wheelL->setFriction(0.9f);
    wheelR->setFriction(0.9f);
    world.addBody(wheelL);
    world.addBody(wheelR);

    // 左轮（下坡侧）+ 右轮（上坡侧）：刚性硬悬挂 + 轮毂马达。
    // collideConnected = false 铁律：车轮与底盘相切重叠，严禁互推
    WheelJointDef defL;
    defL.Initialize(chassis, wheelL, anchorL, Vector2(0.0f, -1.0f));
    defL.enableMotor = true;
    defL.motorSpeed = -12.0f; // 顺时针自转 → 向右（上坡）滚进
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

    float maxAnchorError = 0.0f;
    float maxSteadyPitch = 0.0f; // 稳态段最大俯仰偏差（脱轴/抬头检测）
    float maxSteadyW = 0.0f;     // 稳态段最大角速度（微震发散检测）
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        Vector2 errL = jL->getAnchorB() - jL->getAnchorA();
        Vector2 errR = jR->getAnchorB() - jR->getAnchorA();
        maxAnchorError = std::max(maxAnchorError, std::max(errL.length(), errR.length()));
        if (i >= 60) { // 跳过起步段
            maxSteadyPitch = std::max(maxSteadyPitch, std::abs(chassis->getRotation() - slopeAngle));
            maxSteadyW = std::max(maxSteadyW, std::abs(chassis->getAngularVelocity()));
        }
    }

    Vector2 pos = chassis->getPosition();
    float speed = chassis->getVelocity().length();
    // 战车平稳爬上陡坡：5 秒后沿坡推进 ~24 m（x 方向 +21、y 方向 +6）
    EXPECT_GT(pos.getX(), 12.0f);
    EXPECT_GT(pos.getY(), 6.5f);
    // 巡航速度接近无滑动理论值 w·r = 4.8 m/s
    EXPECT_GT(speed, 3.5f);
    EXPECT_LT(speed, 6.0f);
    // 车轮不脱轴：全程悬挂锚点误差 < 1cm（刚性硬悬挂）
    EXPECT_LT(maxAnchorError, 0.01f);
    // 车体无微震发散：稳态俯仰保持 15°、角速度有界
    EXPECT_LT(maxSteadyPitch, 0.1f);
    EXPECT_LT(maxSteadyW, 1.5f);
    EXPECT_FALSE(std::isnan(pos.getX()));
    EXPECT_FALSE(std::isnan(chassis->getAngularVelocity()));
}
