// GoogleTest 版 V4/004：RopeJoint 绳关节测试
// 场景 1 自由上抛与松弛下垂：绳长 5m、小球在 2m 处上抛 5 m/s，
//   松弛期做完全不受限的抛体运动，绳索冲量恒为 0.000000（零推力干扰）
// 场景 2 坠落急停截断：上升→下坠→绳绷紧急停转单摆，最大伸长 < 0.003m 绝不脱节
// 场景 3 链锤高速甩动：高速切向甩动 300 帧，全程半径 ≤ 5m 圆域，
//   向外甩直拉力饱满、向内运动顺畅滑行（λ≤0 单侧截断自然放行）
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RopeJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>

namespace {
// 三个场景共享的装配：静态天花板 A(0,10) + 1kg 小球 B(0,8) + 5m 绳
// 返回装配好的世界与小球（锚点都在质心，无旋转，局部锚点 = (0,0)）
struct RopeScene {
    World world{ Vector2(0, -9.8f) };
    Body* ball;
    RopeJoint* rope;

    RopeScene(float ballVelX, float ballVelY) {
        Body* anchor = new Body(new Box(0.4f, 0.4f), 0.0f, 10.0f, 0.0f); // 静态天花板
        world.addBody(anchor);

        // 1kg 小球（半径 0.2），出生在 (0,8)：距锚点 2m << 绳长 5m
        float density = 1.0f / (Settings::PAI * 0.04f);
        ball = new Body(new Circle(0.2f), 0.0f, 8.0f, density);
        world.addBody(ball);
        ball->setVelocity(Vector2(ballVelX, ballVelY));

        RopeJointDef def;
        def.bodyA = anchor;
        def.bodyB = ball;
        def.localAnchorA = Vector2(0.0f, 0.0f);
        def.localAnchorB = Vector2(0.0f, 0.0f);
        def.maxLength = 5.0f;
        def.collideConnected = false; // 链锤甩过锚点区域时禁止实体碰撞
        rope = static_cast<RopeJoint*>(world.createJoint(def));
    }
};
} // namespace

// --- 场景 1: 自由上抛与松弛下垂测试 (Slack Free Motion) ---
TEST(V4_004, SlackFreeMotion) {
    RopeScene s(0.0f, 5.0f);
    ASSERT_NE(s.rope, nullptr);
    float dt = 1.0f / 60.0f;

    // 引擎是"先积分后求解"的半隐式欧拉：位置递推 y += v·dt, v += g·dt。
    // 解析轨迹必须按同款递推逐帧推进（不能用连续解析式 y=8+5t-4.9t²，差 O(dt)）
    float vy = 5.0f;
    float analyticY = 8.0f;
    float maxTrajError = 0.0f;
    float maxImpulse = 0.0f;
    int slackFrames = 0;
    for (int i = 0; i < 120; ++i) {
        s.world.step(dt);
        float dist = (s.ball->getPosition() - Vector2(0.0f, 10.0f)).length();
        if (dist >= 5.0f) break; // 绳绷紧（本帧位置修正已介入），松弛期结束
        vy -= 9.8f * dt;
        analyticY += vy * dt;
        maxTrajError = std::max(maxTrajError, std::abs(s.ball->getPosition().getY() - analyticY));
        maxImpulse = std::max(maxImpulse, std::abs(s.rope->getImpulse()));
        ++slackFrames;
    }

    EXPECT_GT(slackFrames, 60);        // 松弛期确实足够长（~87 帧才绷紧）
    EXPECT_LT(maxTrajError, 1e-4f);    // 纯抛体运动：轨迹与解析逐帧吻合
    EXPECT_EQ(maxImpulse, 0.0f);       // 绳索冲量恒为 0.000000，零推力干扰
}

// --- 场景 2: 坠落急停截断测试 (Fall Arrest & Max Length Rigid Stop) ---
TEST(V4_004, FallArrestMaxLength) {
    RopeScene s(0.0f, 5.0f);
    ASSERT_NE(s.rope, nullptr);
    float dt = 1.0f / 60.0f;

    float maxElongation = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        s.world.step(dt);
        float dist = (s.ball->getPosition() - Vector2(0.0f, 10.0f)).length();
        maxElongation = std::max(maxElongation, dist - 5.0f);
        if (std::isnan(dist)) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    // 绳被拉紧截停：最大伸长严格 < 3mm（位置修正当帧拦截，绝不脱节）
    EXPECT_LT(maxElongation, 0.003f);
    // 被稳稳拉住：300 帧后悬挂在锚点正下方绳长处（无横向漂移）
    EXPECT_GT(s.ball->getPosition().getY(), 4.9f);
    EXPECT_LT(std::abs(s.ball->getPosition().getX()), 0.01f);
}

// --- 场景 3: 链锤高速甩动测试 (Flail Orbit Motion) ---
TEST(V4_004, FlailOrbitMotion) {
    RopeScene s(12.0f, 0.0f); // 高速切向初速度
    ASSERT_NE(s.rope, nullptr);
    float dt = 1.0f / 60.0f;

    float maxRadius = 0.0f;
    float maxTension = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        s.world.step(dt);
        float r = (s.ball->getPosition() - Vector2(0.0f, 10.0f)).length();
        maxRadius = std::max(maxRadius, r);
        maxTension = std::max(maxTension, -s.rope->getImpulse()); // 拉力 = -λ (λ≤0 约定)
        if (std::isnan(r)) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxRadius, 5.003f); // 全程保持在半径 ≤ 5m 的圆域内（含冲击帧瞬态余量）
    // 向外甩直时拉力饱满：高速摆动离心力 ~m·v²/r ≈ 29N → 帧冲量 ≥ 0.3 N·s
    EXPECT_GT(maxTension, 0.3f);
}
