// GoogleTest 版 V4/011：CCD 高速时空极限大考（Day 11 专项）
// 场景 1/2 超音速子弹绝对防御：100 m/s 与 150 m/s（单帧位移 1.67~2.5m）射击
//   0.1m 纸片薄墙，CCD 拦截率严格 100%（最大位移不超过墙体厚度一半），法向完美反弹
// 场景 3 CCD 关闭对照：SetBullet(false) 离散检测必定穿墙——证明 CCD 的不可替代性
// 场景 4 45° 斜墙镜面反射：100 m/s 高速下入射角 = 反射角（误差 < 1°），TOI 法线几何绝对准确
// 场景 5 1000 球大落塔零开销回归：CCD 早退守卫生效，CCD 阶段耗时恒 0.00 ms
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Box.h"
#include "Circle.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>
#include <vector>

// --- 场景 1: 100 m/s 超音速子弹绝对防御 (CCD Stress Bullet 100) ---
TEST(V4_011, CCDStressBullet100) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* wall = world.createBox(0.1f, 10.0f, 0.0f, 0.0f, 0.0f); // 0.1m 纸片薄墙
    wall->setRestitution(1.0f);

    Body* bullet = world.createCircle(0.1f, -5.0f, 0.0f, 1.0f);
    bullet->setBullet(true);
    bullet->setRestitution(1.0f);
    bullet->setVelocity(Vector2(100.0f, 0.0f)); // 单帧位移 1.67m

    float maxX = -1e9f;
    bool hasNaN = false;
    for (int i = 0; i < 5; ++i) {
        world.step(dt);
        maxX = std::max(maxX, bullet->getPosition().getX());
        if (std::isnan(bullet->getPosition().getX()) ||
            std::isnan(bullet->getVelocity().getX())) { hasNaN = true; break; }
    }

    // 拦截率 100%：最大位移 < 墙体厚度一半（0.05m），绝对未越过墙体中线
    EXPECT_LT(maxX, 0.05f);
    EXPECT_GT(maxX, -1.0f); // 确实飞到了墙面前缘（而非从未出发）
    // 弹道法向反转：速度精确翻转为 -100（恢复系数 1）
    EXPECT_NEAR(bullet->getVelocity().getX(), -100.0f, 0.5f);
    EXPECT_FALSE(hasNaN);
}

// --- 场景 2: 150 m/s 极限超音速（单帧位移 2.5m） ---
TEST(V4_011, CCDStressBullet150) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* wall = world.createBox(0.1f, 10.0f, 0.0f, 0.0f, 0.0f);
    wall->setRestitution(1.0f);

    Body* bullet = world.createCircle(0.1f, -5.0f, 0.0f, 1.0f);
    bullet->setBullet(true);
    bullet->setRestitution(1.0f);
    bullet->setVelocity(Vector2(150.0f, 0.0f));

    float maxX = -1e9f;
    for (int i = 0; i < 5; ++i) {
        world.step(dt);
        maxX = std::max(maxX, bullet->getPosition().getX());
    }

    EXPECT_LT(maxX, 0.05f); // 150 m/s 下依旧 100% 拦截
    EXPECT_GT(maxX, -1.0f);
    EXPECT_NEAR(bullet->getVelocity().getX(), -150.0f, 0.5f);
    EXPECT_FALSE(std::isnan(bullet->getPosition().getY()));
}

// --- 场景 3: CCD 关闭对照——离散检测必定穿墙 (CCD Disabled Tunneling) ---
// 同一场景 SetBullet(false)：无扫掠轨迹，宽相只查当前紧 AABB。
// 速度选 110 m/s（单帧位移 1.833m）：离散位置序列 -5 → -3.167 → -1.333 → +0.5
// 干净跳过墙体 [-0.05, 0.05]（100 m/s 时离散点恰好落在 0.0 墙内被离散检测抓住）
TEST(V4_011, CCDDisabledTunneling) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* wall = world.createBox(0.1f, 10.0f, 0.0f, 0.0f, 0.0f);
    wall->setRestitution(1.0f);

    Body* bullet = world.createCircle(0.1f, -5.0f, 0.0f, 1.0f);
    bullet->setBullet(false); // 关闭 CCD
    bullet->setRestitution(1.0f);
    bullet->setVelocity(Vector2(110.0f, 0.0f));

    float maxX = -1e9f;
    for (int i = 0; i < 5; ++i) {
        world.step(dt);
        maxX = std::max(maxX, bullet->getPosition().getX());
    }

    // 对比验证：无 CCD 时子弹穿透墙体飞到 +4.2（远超墙体中线 0.05）
    EXPECT_GT(maxX, 1.0f);
    EXPECT_NEAR(bullet->getVelocity().getX(), 110.0f, 0.5f); // 速度毫无损失：完全穿透
}

// --- 场景 4: 45° 斜墙 100 m/s 高速镜面反射 (Slanted Mirror Reflection) ---
TEST(V4_011, SlantedMirrorReflection) {
    World world(Vector2(0.0f, 0.0f)); // 零重力：纯镜面反射
    float dt = 1.0f / 60.0f;

    // -45° 倾斜薄墙（"\" 形），恢复系数 1
    Body* wall = world.createBox(0.1f, 10.0f, 0.0f, 0.0f, 0.0f);
    wall->setRotation(-Settings::PAI / 4.0f);
    wall->setRestitution(1.0f);

    Body* bullet = world.createCircle(0.1f, -5.0f, 0.0f, 1.0f);
    bullet->setBullet(true);
    bullet->setRestitution(1.0f);
    bullet->setVelocity(Vector2(100.0f, 0.0f));

    float maxX = -1e9f;
    for (int i = 0; i < 10; ++i) {
        world.step(dt);
        maxX = std::max(maxX, bullet->getPosition().getX());
    }

    Vector2 v = bullet->getVelocity();
    // 撞击瞬间被拦截（从未越过墙平面）
    EXPECT_LT(maxX, 0.05f);
    EXPECT_GT(maxX, -1.0f);
    // 入射水平向右 → 出射垂直向上（恢复系数 1，TOI 法线几何准确）
    EXPECT_NEAR(v.getX(), 0.0f, 0.1f);
    EXPECT_NEAR(v.getY(), 100.0f, 0.1f);
    // 出射角 = 入射角：出射方向与竖直方向夹角 < 1°（100·sin1° ≈ 1.75 m/s 容差）
    float sinAngle = std::abs(v.getX()) / v.length();
    EXPECT_LT(sinAngle, std::sin(1.0f * Settings::PAI / 180.0f));
    EXPECT_FALSE(std::isnan(v.getX()));
}

// --- 场景 5: 1000 球大落塔零开销回归 (Thousand Ball Zero Overhead) ---
TEST(V4_011, ThousandBallZeroOverhead) {
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // 1000 个普通刚体（全部 SetBullet(false) 默认值）：10 层 × 10 列 × 10 排 球塔
    Body* ground = world.createBox(20.0f, 1.0f, 0.0f, -0.5f, 0.0f);
    const float spacing = 0.51f; // 略大于直径 0.5，出生无初始重叠
    for (int layer = 0; layer < 10; ++layer) {
        for (int row = 0; row < 10; ++row) {
            for (int col = 0; col < 10; ++col) {
                world.createCircle(0.25f,
                    -2.295f + col * spacing, 0.25f + layer * spacing * 0.87f, 1.0f);
            }
        }
    }

    // 球塔崩塌沉降 60 帧
    bool hasNaN = false;
    for (int i = 0; i < 60; ++i) {
        world.step(dt);
        if (std::isnan(ground->getPosition().getX()) ||
            std::isnan(world.getBodies()[500]->getPosition().getY())) { hasNaN = true; break; }
    }
    EXPECT_FALSE(hasNaN);

    // CCD 早退守卫生效的双重证据：
    // ①【铁证】全程 TOI 求解次数恒为 0——零 CCD 计算（早退守卫拦截在求解之前）；
    // ②【计时】CCD 阶段仅剩 O(N) 标志扫描（~0.02ms/1000 清醒刚体，占帧预算 <1%）
    EXPECT_EQ(world.getProfiler().getCounter("CCD TOI Solves"), 0);
    const auto& records = world.getProfiler().GetRecords();
    auto it = records.find("3. CCD");
    ASSERT_NE(it, records.end());
    // 末帧 CCD 区仅剩 O(N) 标志扫描（~0.02ms，即打印值 0.00 ms）。
    // 注意：不做 maxTime 峰值断言——墙钟峰值随机器负载波动（全量并行/后台任务），
    // TOI 求解计数 == 0 才是"零 CCD 计算"的确定性铁证
    EXPECT_LT(it->second.accumulatedTime, 0.05f);
}
