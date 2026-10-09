// GoogleTest 版 V4/005：PulleyJoint 滑轮关节测试
// 场景 1 等比滑轮阿特伍德机平衡：ratio=1、两侧各 2kg，重力拉力完全平衡，
//   两物体悬空静止，速度始终 0.000000
// 场景 2 动滑轮 2:1 机械省力倍率：约束 L = lenA + ratio·lenB 的静力平衡条件是
//   m_B = ratio·m_A（B 侧绳索张力 = ratio × A 侧）——A 挂 1kg 轻物、B 挂 2kg 重物：
//   1×9.8×2 = 2×9.8 完美平衡，零加速度漂移
// 场景 3 运动距离传递比验证：打破平衡（A=2kg、B=1kg），A 下坠 2.0m 时
//   B 被精准提拉上升 1.0m，总绳长恒等式误差 < 1e-4
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "PulleyJoint.h"
#include "Box.h"
#include <cmath>
#include <algorithm>

namespace {
// 滑轮场景装配：A 吊点 (0,10)、B 吊点 (5,10)，重物各吊在吊点正下方 3m
struct PulleyScene {
    World world{ Vector2(0, -9.8f) };
    const Vector2 ga{ 0.0f, 10.0f }; // A 侧滑轮吊点
    const Vector2 gb{ 5.0f, 10.0f }; // B 侧滑轮吊点
    const Vector2 spawnA{ 0.0f, 7.0f };
    const Vector2 spawnB{ 5.0f, 7.0f };
    Body* a; // A 侧重物
    Body* b; // B 侧重物
    PulleyJoint* pulley;

    PulleyScene(float massA, float massB, float ratio) {
        // Box(1,1) 面积 = 1 → 密度 = 质量
        a = new Body(new Box(1.0f, 1.0f), spawnA.getX(), spawnA.getY(), massA);
        b = new Body(new Box(1.0f, 1.0f), spawnB.getX(), spawnB.getY(), massB);
        world.addBody(a);
        world.addBody(b);

        PulleyJointDef def;
        def.bodyA = a;
        def.bodyB = b;
        def.groundAnchorA = ga;
        def.groundAnchorB = gb;
        def.localAnchorA = Vector2(0.0f, 0.0f);
        def.localAnchorB = Vector2(0.0f, 0.0f);
        def.ratio = ratio;
        pulley = static_cast<PulleyJoint*>(world.createJoint(def));
    }

    float lengthA() const { return ga.distance(a->getPosition()); }
    float lengthB() const { return gb.distance(b->getPosition()); }
};
} // namespace

// --- 场景 1: 等比滑轮阿特伍德机平衡测试 (1:1 Ratio Balance) ---
TEST(V4_005, Ratio1Balance) {
    PulleyScene s(2.0f, 2.0f, 1.0f); // 两侧各 2kg，ratio=1
    ASSERT_NE(s.pulley, nullptr);
    float dt = 1.0f / 60.0f;

    float maxSpeed = 0.0f;
    float maxDrop = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        s.world.step(dt);
        maxSpeed = std::max(maxSpeed, std::max(s.a->getVelocity().length(), s.b->getVelocity().length()));
        maxDrop = std::max(maxDrop, std::max(
            (s.a->getPosition() - s.spawnA).length(), (s.b->getPosition() - s.spawnB).length()));
        if (std::isnan(s.a->getPosition().getY())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    // 两侧重力拉力完全平衡：解算后速度逐帧精确归零（热启动+精确解）
    EXPECT_LT(maxSpeed, 1e-4f);
    // 悬空静止：位置被位置修正钉在出生点（漂移 ~1e-6 量级）
    EXPECT_LT(maxDrop, 0.01f);
}

// --- 场景 2: 动滑轮 2:1 机械省力倍率测试 (2:1 Ratio Mechanical Advantage) ---
// 约束 L = lenA + ratio·lenB 的静力平衡条件：T_B = ratio·T_A → m_B = ratio·m_A。
// 2:1 倍率下 A 挂 1kg 轻物、B 挂 2kg 重物：1×9.8×2 = 2×9.8 完美力学平衡。
// 注意：规格原文"A 挂 2kg、B 挂 1kg"在该约束约定下并不平衡（A 会加速下坠）——
// 张力放大在 B 侧，重物应挂 B 侧。
TEST(V4_005, Ratio2MechanicalAdvantage) {
    PulleyScene s(1.0f, 2.0f, 2.0f);
    ASSERT_NE(s.pulley, nullptr);
    float dt = 1.0f / 60.0f;

    float maxSpeed = 0.0f;
    float maxDrift = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        s.world.step(dt);
        maxSpeed = std::max(maxSpeed, std::max(s.a->getVelocity().length(), s.b->getVelocity().length()));
        maxDrift = std::max(maxDrift, std::max(
            (s.a->getPosition() - s.spawnA).length(), (s.b->getPosition() - s.spawnB).length()));
        if (std::isnan(s.b->getPosition().getY())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    // 2:1 完美力学平衡：稳定悬空，零加速度漂移
    EXPECT_LT(maxSpeed, 1e-4f);
    EXPECT_LT(maxDrift, 0.01f);
}

// --- 场景 3: 运动距离传递比验证 (Kinematic Ratio Check) ---
// 打破平衡：A=2kg、B=1kg、ratio=2 → A 加速下坠、B 被提拉上升。
// 运动学恒等式 ΔlenA + ratio·ΔlenB = 0：A 下坠 2.0m 时 B 精确上升 1.0m
TEST(V4_005, KinematicRatioCheck) {
    PulleyScene s(2.0f, 1.0f, 2.0f);
    ASSERT_NE(s.pulley, nullptr);
    float dt = 1.0f / 60.0f;
    float L0 = s.lengthA() + 2.0f * s.lengthB(); // 初始总等效绳长

    float maxIdentityError = 0.0f;
    float dropA = 0.0f;  // A 下坠距离
    float riseB = 0.0f;  // B 上升距离
    bool reached = false;
    for (int i = 0; i < 300; ++i) {
        s.world.step(dt);
        float identity = std::abs(s.lengthA() + 2.0f * s.lengthB() - L0);
        maxIdentityError = std::max(maxIdentityError, identity);
        dropA = s.spawnA.getY() - s.a->getPosition().getY();
        riseB = s.b->getPosition().getY() - s.spawnB.getY();
        if (dropA >= 2.0f) { reached = true; break; }
    }

    EXPECT_TRUE(reached);
    EXPECT_NEAR(dropA, 2.0f, 0.1f);             // A 下坠约 2.0m（帧采样落在帧间）
    EXPECT_NEAR(riseB, dropA * 0.5f, 0.01f);    // B 上升 = A 下坠 × 1/2（2:1 传递比）
    EXPECT_LT(maxIdentityError, 1e-4f);         // 总绳长恒等式误差严格 < 1e-4
}
