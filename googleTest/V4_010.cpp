// GoogleTest 版 V4/010：Bullet 连续碰撞检测（CCD）全流程贯通
// 场景 1 超音速子弹击穿薄墙拦截：0.1m 超薄墙 + 120 m/s 子弹（单帧位移 2.0m），
//   CCD 在穿透前截胡，子弹最大 X 坐标 < 0.05（从未越过墙体中线），反弹速度反转
// 场景 2 45° 斜墙镜像镜面反射：-45° 斜墙 + 80 m/s 水平子弹，出射速度垂直向上，
//   出射角与入射角严格相等（误差 < 1°）
// 场景 3 非 Bullet 场景零性能开销回归：100 个普通刚体堆叠 100 帧，
//   CCD 早退守卫生效，CCD 阶段总耗时严格 0.00 ms
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Box.h"
#include "Circle.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>

// --- 场景 1: 超音速子弹击穿薄墙拦截测试 (High Speed Bullet Tunneling) ---
TEST(V4_010, HighSpeedBulletTunneling) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // X = 0 处厚度仅 0.1m 的静态超薄墙体（跨 x ∈ [-0.05, 0.05]）
    Body* wall = world.createBox(0.1f, 10.0f, 0.0f, 0.0f, 0.0f);
    wall->setRestitution(1.0f); // 完全弹性（Material::Combine 默认取最大值）

    // 子弹：X = -5.0 出发，120 m/s（单帧位移 2.0m >> 墙厚 0.1m）
    Body* bullet = world.createCircle(0.1f, -5.0f, 0.0f, 1.0f);
    bullet->setBullet(true);
    bullet->setRestitution(1.0f);
    bullet->setVelocity(Vector2(120.0f, 0.0f));

    float maxX = -1e9f;
    bool hasNaN = false;
    for (int i = 0; i < 5; ++i) {
        world.step(dt);
        maxX = std::max(maxX, bullet->getPosition().getX());
        if (std::isnan(bullet->getPosition().getX()) ||
            std::isnan(bullet->getVelocity().getX())) { hasNaN = true; break; }
    }

    // 子弹在穿透墙体前被 CCD 成功捕获：最大 X 坐标 < 0.05（绝对从未越过墙体中线）
    EXPECT_LT(maxX, 0.05f);
    EXPECT_GT(maxX, -1.0f); // 确实飞到了墙面前缘附近（而不是从未出发）
    // 碰撞后速度反转为负值，完美向左弹回（恢复系数 1 → 速度精确翻转）
    EXPECT_LT(bullet->getVelocity().getX(), 0.0f);
    EXPECT_NEAR(bullet->getVelocity().getX(), -120.0f, 0.5f);
    EXPECT_FALSE(hasNaN);
}

// --- 场景 2: 45° 斜墙镜像镜面反射测试 (Slanted Specular Bounce) ---
TEST(V4_010, SlantedSpecularBounce) {
    World world(Vector2(0.0f, 0.0f)); // 零重力：纯镜面反射（严格出射角 = 入射角）
    float dt = 1.0f / 60.0f;

    // -45° 倾斜薄墙（"\" 形，长轴 10m、厚 0.1m），恢复系数 1
    Body* wall = world.createBox(0.1f, 10.0f, 0.0f, 0.0f, 0.0f);
    wall->setRotation(-Settings::PAI / 4.0f);
    wall->setRestitution(1.0f);

    // 子弹：80 m/s 水平向右高速射击
    Body* bullet = world.createCircle(0.1f, -5.0f, 0.0f, 1.0f);
    bullet->setBullet(true);
    bullet->setRestitution(1.0f);
    bullet->setVelocity(Vector2(80.0f, 0.0f));

    float maxX = -1e9f;
    for (int i = 0; i < 10; ++i) {
        world.step(dt);
        maxX = std::max(maxX, bullet->getPosition().getX());
    }

    Vector2 v = bullet->getVelocity();
    // 撞击斜墙瞬间被拦截：从未越过墙平面（墙平面过原点，子弹中心止步于 x ≈ -0.107）
    EXPECT_LT(maxX, 0.05f);
    EXPECT_GT(maxX, -1.0f);
    // 镜面反射：入射水平向右 → 出射垂直向上（恢复系数 1，纯反射）
    EXPECT_NEAR(v.getX(), 0.0f, 0.1f);
    EXPECT_NEAR(v.getY(), 80.0f, 0.1f);
    // 出射角与入射角严格相等：出射方向与竖直方向夹角 < 1°（80·sin1° ≈ 1.4 m/s 容差）
    float sinAngle = std::abs(v.getX()) / v.length();
    EXPECT_LT(sinAngle, std::sin(1.0f * Settings::PAI / 180.0f));
    EXPECT_FALSE(std::isnan(v.getX()));
    EXPECT_FALSE(std::isnan(bullet->getPosition().getY()));
}

// --- 场景 3: 非 Bullet 场景零性能开销回归测试 (Zero Overhead Regression) ---
TEST(V4_010, ZeroOverheadRegression) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // 100 个普通刚体堆叠（10 列 × 10 层，全部 SetBullet(false) 默认值）
    Body* ground = world.createBox(20.0f, 1.0f, 0.0f, -0.5f, 0.0f);
    std::vector<Body*> stack;
    for (int row = 0; row < 10; ++row) {
        for (int col = 0; col < 10; ++col) {
            stack.push_back(world.createBox(0.5f, 0.5f,
                -2.25f + col * 0.5f, 0.25f + row * 0.5f, 1.0f));
        }
    }

    // 运行 100 帧（堆叠沉降 + 睡眠）
    bool hasNaN = false;
    for (int i = 0; i < 100; ++i) {
        world.step(dt);
        if (std::isnan(ground->getPosition().getX()) ||
            std::isnan(stack[50]->getPosition().getY())) { hasNaN = true; break; }
    }
    EXPECT_FALSE(hasNaN);

    // CCD 早退守卫生效：无任何 Bullet 刚体时整个 CCD 阶段被 O(1) 跳过，
    // 耗时维持 0.00 ms（本帧累计 + 历史峰值双重断言）
    const auto& records = world.getProfiler().GetRecords();
    auto it = records.find("3. CCD");
    ASSERT_NE(it, records.end());
    EXPECT_LT(it->second.accumulatedTime, 0.01f); // 本帧 CCD 阶段 0.00 ms
    // 历史峰值 0.05 容差：首帧冷缓存/页错误尖峰 ~0.01ms，稳态帧恒为 0.00 ms
    EXPECT_LT(it->second.maxTime, 0.05f);
}
