// GoogleTest 版 V4/002：PrismaticJoint 滑块关节测试
// 场景 1 自由滑块纯平移：水平滑轨 + 10 m/s 初速，只滑不沉、绝不翻滚，
//   且无 Baumgarte 阻尼（滑行速度不被耗掉）
// 场景 2 45° 斜轨限位反弹：重力滑落冲击 -2.0 下限位，被硬生生挡停弹起，
//   全程位移受控在 [-2.0, +2.0]，超界 < 0.005 m
// 场景 3 等限位锁死：lower=upper=1.5，1000N 巨大外力交替推拉纹丝不动，
//   完全退化为刚性连接
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "PrismaticJoint.h"
#include "Box.h"
#include <cmath>
#include <algorithm>

// --- 场景 1: 自由滑块纯平移测试 (Pure Linear Sliding) ---
// 静态基座 A(0,0)，滑轨水平向右 t=(1,0)；1kg 箱子 B 挂上轨道、初速 10 m/s。
// 重力只拉垂向——垂向约束必须把箱子钉死在 y=0 线上，轴向自由度完全自由
TEST(V4_002, PureLinearSliding) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* rail = new Body(new Box(0.4f, 0.4f), 0.0f, 0.0f, 0.0f); // 静态轨道基座
    world.addBody(rail);

    Body* box = new Body(new Box(1.0f, 1.0f), 2.0f, 0.0f, 1.0f); // 1kg 滑块
    world.addBody(box);
    box->setVelocity(Vector2(10.0f, 0.0f));

    PrismaticJointDef def;
    def.bodyA = rail;
    def.bodyB = box;
    def.localAnchorA = Vector2(0.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.localAxisA = Vector2(1.0f, 0.0f); // 水平滑轨
    def.referenceAngle = 0.0f;
    PrismaticJoint* joint = static_cast<PrismaticJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    float maxYDrift = 0.0f;
    float maxRot = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        maxYDrift = std::max(maxYDrift, std::abs(box->getPosition().getY() - 0.0f));
        maxRot = std::max(maxRot, std::abs(box->getRotation()));
        if (std::isnan(box->getPosition().getX())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxYDrift, 1e-4f); // 垂直位移漂移 < 10^-4 m（只滑不沉）
    EXPECT_LT(maxRot, 1e-5f);    // 旋转 < 10^-5 rad（绝不翻滚）
    // 无速度偏置设计的验证：300 帧后滑行速度分毫未损（有偏置会被"阻尼"耗掉）
    EXPECT_NEAR(box->getVelocity().getX(), 10.0f, 0.01f);
}

// --- 场景 2: 45° 斜向滑轨限位反弹测试 (Slanted Rail Limits) ---
// 滑轨 45° 斜向上 t=(cos45, sin45)，滑动范围 [-2, +2]；滑块从位移 0 出发，
// 重力沿轨分量 -6.93 m/s² 加速滑落，~5.4 m/s 冲击 -2.0 下限位。
// 限位冲量当帧挡停并靠 Baumgarte 偏置弹回（塑性限位，弹幅逐次衰减收敛）
TEST(V4_002, SlantedRailLimits) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* rail = new Body(new Box(0.4f, 0.4f), 0.0f, 0.0f, 0.0f);
    world.addBody(rail);

    Body* box = new Body(new Box(1.0f, 1.0f), 0.0f, 0.0f, 1.0f); // 出生在位移 0
    world.addBody(box);

    const float SQ2 = 0.70710678f;
    PrismaticJointDef def;
    def.bodyA = rail;
    def.bodyB = box;
    def.localAnchorA = Vector2(0.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.localAxisA = Vector2(SQ2, SQ2); // 45° 斜轨
    def.referenceAngle = 0.0f;
    def.enableLimit = true;
    def.lowerTranslation = -2.0f;
    def.upperTranslation = +2.0f;
    def.collideConnected = false; // 滑块出生时与基座重叠，必须过滤实体接触
    PrismaticJoint* joint = static_cast<PrismaticJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    Vector2 axis(SQ2, SQ2);
    float minTranslation = 0.0f;
    float maxTranslation = 0.0f;
    float maxAxialVelAfterHit = -1e9f;
    bool hit = false;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        float trans = joint->getJointTranslation();
        minTranslation = std::min(minTranslation, trans);
        maxTranslation = std::max(maxTranslation, trans);
        float axialVel = axis.dot(box->getVelocity());
        if (!hit && trans < -1.9f) hit = true; // 首次冲击下限位
        if (hit) maxAxialVelAfterHit = std::max(maxAxialVelAfterHit, axialVel);
        if (std::isnan(trans)) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_GT(minTranslation, -2.005f); // 超界误差 < 0.005 m：位置修正当帧拦截越界
    EXPECT_LT(maxTranslation, +2.005f); // 全程位移严格受控在 [-2, +2]
    EXPECT_GT(maxAxialVelAfterHit, 0.1f); // 被硬生生挡停弹起：轴向速度从 -5.4 弹回正值
    EXPECT_LT(std::abs(box->getRotation()), 1e-4f); // 只滑不转
    // 弹幅逐次衰减，300 帧后停靠在下限位
    EXPECT_NEAR(joint->getJointTranslation(), -2.0f, 0.005f);
}

// --- 场景 3: 锁死限位测试 (Equal Limits Lock) ---
// lowerTranslation = upperTranslation = 1.5：限位退化为双向等式约束。
// 1000N 巨大外力交替推拉（滑块自重仅 9.8N），滑块必须牢牢焊死在 1.5 处
TEST(V4_002, EqualLimitsLock) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* rail = new Body(new Box(0.4f, 0.4f), 0.0f, 0.0f, 0.0f);
    world.addBody(rail);

    Body* box = new Body(new Box(1.0f, 1.0f), 1.5f, 0.0f, 1.0f); // 装配在位移 1.5
    world.addBody(box);

    PrismaticJointDef def;
    def.bodyA = rail;
    def.bodyB = box;
    def.localAnchorA = Vector2(0.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.localAxisA = Vector2(1.0f, 0.0f);
    def.referenceAngle = 0.0f;
    def.enableLimit = true;
    def.lowerTranslation = 1.5f;
    def.upperTranslation = 1.5f; // 等限位：双向锁死
    def.collideConnected = false;
    PrismaticJoint* joint = static_cast<PrismaticJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    float maxTranslationError = 0.0f;
    float maxRot = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        // 每 75 帧换一次方向：+1000N 推 / -1000N 拉
        float dir = ((i / 75) % 2 == 0) ? 1.0f : -1.0f;
        box->applyForceAtPoint(Vector2(dir * 1000.0f, 0.0f), box->getPosition());
        world.step(dt);
        maxTranslationError = std::max(maxTranslationError, std::abs(joint->getJointTranslation() - 1.5f));
        maxRot = std::max(maxRot, std::abs(box->getRotation()));
        if (std::isnan(box->getPosition().getX())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxTranslationError, 0.005f); // 1.5m 处牢牢焊死：1000N 推拉纹丝不动
    EXPECT_LT(maxRot, 1e-4f);              // 不旋转
}
