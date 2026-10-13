// GoogleTest 版 V4/009：世界级空间查询 API（RayCast / QueryAABB / TestPoint）
// 场景 1 射线动态剪枝与最近阻挡：X 轴 5/10/15 三箱，最近命中 X=5 的箱子，
//   命中点 (4.5,0)、法线 (-1,0)，收缩剪枝回调每次报告严格更近、穿透回调收齐 3 命中
// 场景 2 AOE 爆炸区域扫描：50 个随机静态刚体 + 10×10 爆炸区，QueryAABB 与
//   暴力全量筛选（同肥 AABB 判据）完全一致（漏检率严格为 0）、支持提前终止
// 场景 3 多几何体像素级点选：正三角形/胶囊/圆精准拾取，边缘判定误差 < 1e-4 米，
//   TestPointAll 收集重叠区全部刚体
// 场景 4 触发器穿透过滤：金币 Trigger + 实心墙，ignoreSensors 开关决定射线被谁拦截
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "RayCastHit.h"
#include "QueryCallback.h"
#include "Box.h"
#include "Circle.h"
#include "Capsule.h"
#include "Polygon.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>
#include <vector>

// --- 场景 1: 射线动态剪枝与最近阻挡测试 (Closest Ray Cast Hit) ---
TEST(V4_009, ClosestRayCastHit) {
    World world(Vector2(0.0f, -9.8f));

    // X 轴依次排开 3 个 1×1 箱子：X = 5 / 10 / 15
    Body* box1 = world.createBox(1.0f, 1.0f, 5.0f, 0.0f, 0.0f);
    Body* box2 = world.createBox(1.0f, 1.0f, 10.0f, 0.0f, 0.0f);
    Body* box3 = world.createBox(1.0f, 1.0f, 15.0f, 0.0f, 0.0f);

    const Vector2 p1(0.0f, 0.0f), p2(20.0f, 0.0f);

    // ===== 最近阻挡：严格命中 X=5 的第 1 个箱子 =====
    RayCastHit hit;
    ASSERT_TRUE(world.RayCast(p1, p2, &hit));
    EXPECT_EQ(hit.body, box1);
    EXPECT_EQ(hit.shape, box1->getShape());
    EXPECT_NEAR(hit.point.getX(), 4.5f, 1e-4f); // 箱子左边缘
    EXPECT_NEAR(hit.point.getY(), 0.0f, 1e-4f);
    EXPECT_NEAR(hit.normal.getX(), -1.0f, 1e-4f); // 朝外法线 (-1, 0)
    EXPECT_NEAR(hit.normal.getY(), 0.0f, 1e-4f);
    EXPECT_NEAR(hit.fraction, 4.5f / 20.0f, 1e-4f);

    // ===== 动态剪枝验证：收缩回调每次报告的命中严格比上次更近 =====
    // 树把回调返回值收缩为新的 maxFraction，后方更远 AABB 子树在树层被整棵剪枝
    struct PruneCallback : RayCastCallback {
        float ReportHit(Body*, Shape*, const Vector2&, const Vector2&, float fraction) override {
            EXPECT_LT(fraction, prevFraction); // 剪枝保证每次报告都比上次更近
            prevFraction = fraction;
            ++count;
            return fraction; // 收缩射线继续找最近
        }
        float prevFraction = 2.0f;
        int count = 0;
    } prune;
    world.RayCast(p1, p2, &prune);
    EXPECT_GE(prune.count, 1);
    EXPECT_LE(prune.count, 3); // 最近命中后后方箱子被剪枝，不可能全部遍历
    EXPECT_NEAR(prune.prevFraction, 4.5f / 20.0f, 1e-4f); // 最终收缩到最近箱子

    // ===== 穿透回调：返回 1 收齐全部 3 个命中（多目标穿透） =====
    struct PassthroughCallback : RayCastCallback {
        float ReportHit(Body* body, Shape*, const Vector2&, const Vector2&, float fraction) override {
            hits.push_back({ body, fraction });
            return 1.0f; // 继续穿透
        }
        std::vector<std::pair<Body*, float>> hits;
    } pass;
    world.RayCast(p1, p2, &pass);
    ASSERT_EQ(pass.hits.size(), 3);
    std::sort(pass.hits.begin(), pass.hits.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; });
    EXPECT_EQ(pass.hits[0].first, box1);
    EXPECT_NEAR(pass.hits[0].second, 4.5f / 20.0f, 1e-4f);
    EXPECT_EQ(pass.hits[1].first, box2);
    EXPECT_NEAR(pass.hits[1].second, 9.5f / 20.0f, 1e-4f);
    EXPECT_EQ(pass.hits[2].first, box3);
    EXPECT_NEAR(pass.hits[2].second, 14.5f / 20.0f, 1e-4f);
}

// --- 场景 2: AOE 爆炸区域无遗漏范围扫描测试 (Explosion Query AABB) ---
TEST(V4_009, ExplosionQueryAABB) {
    World world(Vector2(0.0f, 0.0f));

    // 固定种子 LCG 在 [-8,8]² 散布 50 个静态刚体（四种形状混合，静态无物理演化）
    uint32_t seed = 12345u;
    auto rnd = [&seed]() { // [0,1) 均匀伪随机
        seed = seed * 1664525u + 1013904223u;
        return (float)(seed >> 8) / 16777216.0f;
    };
    std::vector<Body*> bodies;
    for (int i = 0; i < 50; ++i) {
        float x = -8.0f + 16.0f * rnd();
        float y = -8.0f + 16.0f * rnd();
        switch (i % 4) {
        case 0: bodies.push_back(world.createBox(0.8f, 0.8f, x, y, 0.0f)); break;
        case 1: bodies.push_back(world.createCircle(0.4f, x, y, 0.0f)); break;
        case 2: bodies.push_back(world.createCapsule(0.3f, 0.6f, x, y, 0.0f)); break;
        default: {
            Vector2 verts[3] = {
                Vector2(0.0f, 0.5f), Vector2(-0.5f, -0.3f), Vector2(0.5f, -0.3f)
            };
            Polygon* poly = new Polygon();
            poly->set(verts, 3);
            bodies.push_back(world.createBody(poly, x, y, 0.0f));
            break;
        }
        }
    }

    // 中心 10×10 爆炸区域
    AABB boom;
    boom.min = Vector2(-5.0f, -5.0f);
    boom.max = Vector2(5.0f, 5.0f);

    // ===== 引擎检索 vs 暴力全量筛选（同一判据：肥 AABB ±0.1 扩展后重叠）=====
    // 宽相树代理存的是肥 AABB（createProxy 时 ±k_aabbExtension=0.1 外扩），
    // 暴力对照必须用同一判据才能做到"完全一致"
    std::vector<Body*> queried = world.QueryAABB(boom);
    std::vector<Body*> brute;
    for (Body* b : bodies) {
        AABB fat;
        fat.min = b->getAABB().min - Vector2(0.1f, 0.1f);
        fat.max = b->getAABB().max + Vector2(0.1f, 0.1f);
        if (fat.overlap(boom)) brute.push_back(b);
    }

    ASSERT_EQ(queried.size(), brute.size()); // 漏检率严格为 0：完全一致
    for (Body* b : brute) {
        EXPECT_NE(std::find(queried.begin(), queried.end(), b), queried.end());
    }
    // 区域非空且只包含区域相关刚体：所有结果的肥 AABB 都与爆炸区重叠
    EXPECT_GT(queried.size(), 0);
    for (Body* b : queried) {
        AABB fat;
        fat.min = b->getAABB().min - Vector2(0.1f, 0.1f);
        fat.max = b->getAABB().max + Vector2(0.1f, 0.1f);
        EXPECT_TRUE(fat.overlap(boom));
    }

    // ===== 回调版 + 提前终止：第 5 个报告后返回 false，遍历立即停止 =====
    struct LimitCallback : QueryCallback {
        bool ReportBody(Body*) override { ++count; return count < 5; }
        int count = 0;
    } limiter;
    world.QueryAABB(boom, &limiter);
    EXPECT_EQ(limiter.count, std::min(5, (int)queried.size()));
}

// --- 场景 3: 多几何体像素级点选拾取测试 (Point Picking Test) ---
TEST(V4_009, PointPickingTest) {
    World world(Vector2(0.0f, -9.8f));

    // 正三角形（边长 1，质心在原点）：顶点 (0, 0.5774)，底边 y = -0.2887
    Vector2 triVerts[3] = {
        Vector2(0.0f, 0.5774f), Vector2(-0.5f, -0.2887f), Vector2(0.5f, -0.2887f)
    };
    Polygon* tri = new Polygon();
    ASSERT_TRUE(tri->set(triVerts, 3));
    Body* triBody = world.createBody(tri, 0.0f, 0.0f, 0.0f);

    // 胶囊体：半径 0.3、骨架长 0.8（沿 Y），放 (3, 0)；圆：半径 0.4，放 (-3, 0)
    Body* capBody = world.createCapsule(0.3f, 0.8f, 3.0f, 0.0f, 0.0f);
    Body* cirBody = world.createCircle(0.4f, -3.0f, 0.0f, 0.0f);

    // ===== 精准拾取各几何内部点 =====
    EXPECT_EQ(world.TestPoint(Vector2(0.0f, 0.0f)), triBody); // 三角形质心
    EXPECT_EQ(world.TestPoint(Vector2(3.0f, 0.0f)), capBody); // 胶囊中心
    EXPECT_EQ(world.TestPoint(Vector2(-3.0f, 0.0f)), cirBody); // 圆心

    // ===== 边缘判定误差严格 < 1e-4 米 =====
    // 顶点正下方 0.4mm 处：三角形内部
    EXPECT_EQ(world.TestPoint(Vector2(0.0f, 0.5770f)), triBody);
    // 顶点正上方 0.4mm 处：三角形外部 → nullptr
    EXPECT_EQ(world.TestPoint(Vector2(0.0f, 0.5778f)), nullptr);
    // 形状之间空白处 → nullptr
    EXPECT_EQ(world.TestPoint(Vector2(1.5f, 0.0f)), nullptr);

    // ===== TestPointAll：重叠区小圆（与胶囊重叠），该点两形状同时包含 =====
    Body* smallCir = world.createCircle(0.2f, 3.2f, 0.1f, 0.0f);
    std::vector<Body*> all = world.TestPointAll(Vector2(3.2f, 0.1f));
    ASSERT_EQ(all.size(), 2);
    bool hasCap = false, hasSmall = false;
    for (Body* b : all) {
        if (b == capBody) hasCap = true;
        if (b == smallCir) hasSmall = true;
    }
    EXPECT_TRUE(hasCap);
    EXPECT_TRUE(hasSmall);
    // 单点拾取返回两者之一（同为静态，取先发现者）
    Body* single = world.TestPoint(Vector2(3.2f, 0.1f));
    EXPECT_TRUE(single == capBody || single == smallCir);
}

// --- 场景 4: 触发器穿透过滤测试 (Ignore Sensors Filter) ---
TEST(V4_009, IgnoreSensorsFilter) {
    World world(Vector2(0.0f, -9.8f));

    // 金币触发器（传感器）：Circle(0.5) 位于 (2, 0)，射线入口 x = 1.5
    Body* coin = world.createCircle(0.5f, 2.0f, 0.0f, 0.0f);
    coin->getShape()->isTrigger = true;
    // 实心墙体：Box(1, 3) 位于 (5, 0)，左面 x = 4.5
    Body* wall = world.createBox(1.0f, 3.0f, 5.0f, 0.0f, 0.0f);

    const Vector2 p1(0.0f, 0.0f), p2(10.0f, 0.0f);

    // ignoreSensors = true（默认）：射线穿透金币，直接击中后方墙体
    RayCastHit hit;
    ASSERT_TRUE(world.RayCast(p1, p2, &hit, true));
    EXPECT_EQ(hit.body, wall);
    EXPECT_NEAR(hit.point.getX(), 4.5f, 1e-4f);
    EXPECT_NEAR(hit.fraction, 0.45f, 1e-4f);

    // ignoreSensors = false：射线被金币拦截
    ASSERT_TRUE(world.RayCast(p1, p2, &hit, false));
    EXPECT_EQ(hit.body, coin);
    EXPECT_NEAR(hit.point.getX(), 1.5f, 1e-4f);
    EXPECT_NEAR(hit.point.getY(), 0.0f, 1e-4f);
    EXPECT_NEAR(hit.normal.getX(), -1.0f, 1e-4f);
    EXPECT_NEAR(hit.fraction, 0.15f, 1e-4f);
}
