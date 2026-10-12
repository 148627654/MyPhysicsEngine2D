// GoogleTest 版 V4/008：MouseJoint 鼠标关节测试
// 场景 1 静态目标点平滑吸附：1kg 小球被鼠标关节从 (0,0) 拖向 (3,4)，软弹簧
//   平滑吸附、无高频抖动，最终在目标点下方 1cm（重力悬挂 sag = mg/k）稳定悬停
// 场景 2 瞬间大跨度瞬移防爆炸：目标点从 (3,4) 瞬移到 (100,4)（跨越 97 米），
//   最大输出拉力被库仑圆盘限幅精确锁死在 Fmax=1000N，小球受控加速零崩溃
// 场景 3 拖拽释放动量保留：零重力世界高速匀速移动目标点（20 m/s），释放关节后
//   小球沿切线惯性飞出，动量连续守恒
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "MouseJoint.h"
#include "Box.h"
#include "Circle.h"
#include <cmath>
#include <algorithm>

namespace {
// 1kg 小球 + 远处静态地面挂点（bodyA）+ 鼠标关节装配（抓取点 = 球心）
struct MouseDragScene {
    World world{ Vector2(0, -9.8f) };
    float dt = 1.0f / 60.0f;
    Body* ground;
    Body* ball;
    MouseJoint* joint;

    MouseDragScene() {
        ground = new Body(new Box(0.4f, 0.4f), 0.0f, -10.0f, 0.0f); // 静态地面刚体（虚设 bodyA）
        world.addBody(ground);
        // 半径 0.5 圆球，密度 1/π·0.25 → 质量 ≈ 1.0 kg
        float density = 1.0f / (Settings::PAI * 0.25f);
        ball = new Body(new Circle(0.5f), 0.0f, 0.0f, density);
        world.addBody(ball);

        MouseJointDef def;
        def.bodyA = ground;
        def.bodyB = ball;
        def.target = Vector2(3.0f, 4.0f); // 鼠标初始目标点
        def.localAnchorB = Vector2(0.0f, 0.0f); // 抓取点 = 球心（装配时鼠标点在球上）
        def.maxForce = 10000.0f;  // 远大于最大弹簧力 ~4935N：纯软弹簧不触发限幅
        def.collideConnected = false;
        joint = static_cast<MouseJoint*>(world.createJoint(def));
    }
};
} // namespace

// --- 场景 1: 静态目标点平滑吸附测试 (Static Target Tracking) ---
// 【软约束稳态观测约定】本引擎"先积分后求解"：悬停稳态下每帧重力先注入 -g·dt 的
// 速度，求解器再注入等量反向冲量精确抵消——刚体位置完全冻结，但"求解后速度"
// 读数恒等于每帧重力注入量 g·dt ≈ 0.1633 m/s（Box2D 鼠标关节同款现象）。
// 因此验收断言真实运动量：位置每帧漂移 + 速度逐帧跳变，而非观测速度归零
TEST(V4_008, StaticTargetTracking) {
    MouseDragScene s;

    float maxDrift = 0.0f;   // 稳定段每帧位置位移（真实运动量）
    float maxVelStep = 0.0f; // 稳定段每帧速度跳变（抖动检测）
    Vector2 prevPos = s.ball->getPosition();
    Vector2 prevVel = s.ball->getVelocity();
    for (int i = 0; i < 60; ++i) {
        s.world.step(s.dt);
        Vector2 pos = s.ball->getPosition();
        Vector2 vel = s.ball->getVelocity();
        if (i >= 40) { // 稳定悬停段
            maxDrift = std::max(maxDrift, (pos - prevPos).length());
            maxVelStep = std::max(maxVelStep, (vel - prevVel).length());
        }
        prevPos = pos;
        prevVel = vel;
    }

    Vector2 pos = s.ball->getPosition();
    Vector2 vel = s.ball->getVelocity();
    // 平滑飞向 (3,4) 并在其下方稳定悬停
    // （弹簧 sag = (1+γ/K)·g·dt/β ≈ 2cm，在目标点正下方）
    EXPECT_NEAR(pos.getX(), 3.0f, 0.05f);
    EXPECT_NEAR(pos.getY(), 4.0f, 0.05f);
    // 位置严格冻结：稳定段每帧漂移 < 1e-4 m（真实运动量衰减至零）
    EXPECT_LT(maxDrift, 1e-4f);
    // 无高频抖动：速度逐帧跳变收敛到 0（速度收敛到恒定 g·dt，无振荡）
    EXPECT_LT(maxVelStep, 1e-3f);
    // 观测速度收敛到稳态值：横向归零、垂向 = 每帧重力注入量 g·dt
    EXPECT_LT(std::abs(vel.getX()), 0.01f);
    EXPECT_NEAR(vel.getY(), 9.8f * s.dt, 0.01f);
    EXPECT_FALSE(std::isnan(pos.getX()));
    EXPECT_FALSE(std::isnan(vel.getY()));
}

// --- 场景 2: 瞬间大跨度瞬移防爆炸测试 (Teleportation Anti Explosion) ---
TEST(V4_008, TeleportationAntiExplosion) {
    MouseDragScene s;
    s.joint->setMaxForce(1000.0f); // 规格推荐值：1000 × bodyB 质量（1kg）

    // 稳定吸附阶段（40 帧：小球已在 (3,4) 附近悬停）
    for (int i = 0; i < 40; ++i) s.world.step(s.dt);

    // 鼠标突发"瞬移"：目标点瞬间跨越 97 米到 (100,4)。
    // 交互惯例：外部输入应唤醒被拖刚体（否则可能处于睡眠状态）
    s.joint->setTarget(Vector2(100.0f, 4.0f));
    s.ball->setAwake(true);

    float maxForceObs = 0.0f;
    float maxSpeed = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 180; ++i) {
        s.world.step(s.dt);
        Vector2 f = s.joint->getForce(s.dt);
        maxForceObs = std::max(maxForceObs, f.length());
        maxSpeed = std::max(maxSpeed, s.ball->getVelocity().length());
        if (std::isnan(s.ball->getPosition().getX()) ||
            std::isnan(s.ball->getVelocity().getX())) { hasNaN = true; break; }
    }

    // 最大输出拉力被精确锁死在 Fmax = 1000 N 以内（库仑圆盘向量限幅）
    EXPECT_LE(maxForceObs, 1000.0f + 0.01f);
    // 小球以受控加速度前进（峰值速度 ~440 m/s 远低于发散值），零崩溃零 NaN
    EXPECT_LT(maxSpeed, 1000.0f);
    EXPECT_FALSE(hasNaN);
    // 瞬移后小球最终抵达新目标点附近稳定悬停
    Vector2 pos = s.ball->getPosition();
    EXPECT_NEAR(pos.getX(), 100.0f, 0.1f);
    EXPECT_NEAR(pos.getY(), 4.0f, 0.1f);
}

// --- 场景 3: 拖拽释放动量保留测试 (Drag Release Throw) ---
// 零重力世界：目标点以 20 m/s 匀速右移，小球软弹簧跟随后恒定滞后
// C = -v/β ≈ -1.22 m（软约束稳态：λ=0 时锚点速度 v = -β·C）。
// 释放关节后无任何外力，小球沿切线匀速惯性飞出，动量连续守恒
TEST(V4_008, DragReleaseThrow) {
    World world(Vector2(0.0f, 0.0f)); // 零重力：释放后动量严格守恒
    float dt = 1.0f / 60.0f;

    Body* ground = new Body(new Box(0.4f, 0.4f), -5.0f, -5.0f, 0.0f);
    world.addBody(ground);
    float density = 1.0f / (Settings::PAI * 0.25f);
    Body* ball = new Body(new Circle(0.5f), 0.0f, 0.0f, density);
    world.addBody(ball);

    MouseJointDef def;
    def.bodyA = ground;
    def.bodyB = ball;
    def.target = Vector2(0.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.maxForce = 10000.0f;
    def.collideConnected = false;
    MouseJoint* joint = static_cast<MouseJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    // 高速匀速拖动 2 秒：目标点每帧右移 20·dt，小球被带动跟随飞行
    Vector2 target(0.0f, 0.0f);
    for (int i = 0; i < 120; ++i) {
        target = target + Vector2(20.0f * dt, 0.0f);
        joint->setTarget(target);
        world.step(dt);
    }

    // 释放瞬间：记录动量（速度）与位置
    Vector2 vBefore = ball->getVelocity();
    Vector2 posBefore = ball->getPosition();
    EXPECT_NEAR(vBefore.getX(), 20.0f, 0.1f); // 已跟上 20 m/s 拖动速度
    EXPECT_LT(std::abs(vBefore.getY()), 0.01f); // 无横向漂移
    EXPECT_GT(posBefore.getX(), 35.0f); // 全程紧跟目标点（滞后仅 ~1.2 m）

    // 释放鼠标关节（玩家松手）
    world.destroyJoint(joint);

    // 继续模拟 0.5 秒：无外力，应严格沿切线惯性飞出
    for (int i = 0; i < 30; ++i) world.step(dt);

    Vector2 vAfter = ball->getVelocity();
    Vector2 posAfter = ball->getPosition();
    // 动量连续守恒：释放前后速度严格不变（零重力下无任何冲量源）
    EXPECT_LT((vAfter - vBefore).length(), 1e-6f);
    // 沿切线直线飞行：位置 = 释放点 + v·t
    Vector2 expected = posBefore + vBefore * (30.0f * dt);
    EXPECT_LT((posAfter - expected).length(), 0.01f);
    EXPECT_FALSE(std::isnan(posAfter.getX()));
}
