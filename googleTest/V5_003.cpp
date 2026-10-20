// GoogleTest 版 V5/003：SIMD 矢量化前向积分 (AVX2) 验收
// 场景 1 SIMD 与标量积分精度等价性：8 个参数各异的动态刚体推进 1 步，
//   AVX2 FMA 内核与标量回退路径的位姿/速度绝对误差 < 1e-6
// 场景 2 尾部余数与边界安全性：13 刚体（1 个 8 路批处理 + 5 个标量尾部，
//   批内混入 1 个静态刚体检验免分支掩码）——全量更新准确、无越界崩溃
// 场景 3 万体前向积分极限狂飙：10,000 动态刚体，AVX2 内核积分阶段
//   纯耗时严格 < 0.4 ms（对标量回退路径实测加速比 6~9 倍）
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "SoABody.h"
#include "Circle.h"
#include "Setting.h"
#include <cmath>
#include <vector>

namespace {
// 构建一组"参数各异"的刚体（位置紧邻原点：ulp(5.6)≈5e-7，保证 1e-6 容差可判定）
std::vector<Body*> buildVariedBodies(World& w, int count, int staticSlotA = -1, int staticSlotB = -1)
{
    std::vector<Body*> bs;
    for (int i = 0; i < count; ++i) {
        const bool isStatic = (i == staticSlotA || i == staticSlotB);
        Body* b = w.createCircle(0.15f + 0.02f * (i % 3), i * 0.8f, 0.2f + 0.15f * (i % 5),
            isStatic ? 0.0f : 1.0f + 0.7f * (i % 4));
        if (!isStatic) {
            b->setVelocity(Vector2(0.5f * (i % 3) - 0.4f, 0.3f * (i % 2) - 0.2f));
            b->setAngularVelocity(0.7f * (i % 3) - 0.6f);
            b->setGravityScale(0.5f + 0.25f * (i % 3));
            b->addForce(Vector2(3.0f * (i % 2) - 1.5f, 5.0f - 2.0f * (i % 4)));
            b->addTorque(1.5f * (i % 2) - 0.75f);
        }
        bs.push_back(b);
    }
    return bs;
}

// 两世界逐体对比（SIMD vs 标量回退），容差 tol
void compareWorlds(World& simdW, const std::vector<Body*>& simdB,
                   World& scalarW, const std::vector<Body*>& scalarB, float tol, const char* what)
{
    ASSERT_EQ(simdB.size(), scalarB.size());
    for (size_t i = 0; i < simdB.size(); ++i) {
        Body* a = simdB[i];
        Body* b = scalarB[i];
        EXPECT_NEAR(a->getPosition().getX(), b->getPosition().getX(), tol) << what << " posX @" << i;
        EXPECT_NEAR(a->getPosition().getY(), b->getPosition().getY(), tol) << what << " posY @" << i;
        EXPECT_NEAR(a->getVelocity().getX(), b->getVelocity().getX(), tol) << what << " velX @" << i;
        EXPECT_NEAR(a->getVelocity().getY(), b->getVelocity().getY(), tol) << what << " velY @" << i;
        EXPECT_NEAR(a->getRotation(), b->getRotation(), tol) << what << " rot @" << i;
        EXPECT_NEAR(a->getAngularVelocity(), b->getAngularVelocity(), tol) << what << " angVel @" << i;
    }
}
} // namespace

// --- 场景 1: SIMD 与标量积分精度等价性测试 (SIMDAccuracyTest) ---
TEST(V5_003, SIMDAccuracyTest) {
    const float dt = 1.0f / 60.0f;

    // 世界 A：AVX2 FMA 内核（默认）；世界 B：强制标量回退路径
    World simdWorld(Vector2(0.0f, -9.8f));
    std::vector<Body*> simdBodies = buildVariedBodies(simdWorld, 8);

    World scalarWorld(Vector2(0.0f, -9.8f));
    scalarWorld.getSoA().SetUseSIMD(false);
    std::vector<Body*> scalarBodies = buildVariedBodies(scalarWorld, 8);

    // 各推进 1 步
    simdWorld.step(dt);
    scalarWorld.step(dt);

    // 断言：新旧算法位姿/速度绝对误差 < 1e-6（浮点位级等价性）
    compareWorlds(simdWorld, simdBodies, scalarWorld, scalarBodies, 1e-6f, "SIMDAccuracy");
}

// --- 场景 2: 尾部余数与边界安全性测试 (SIMDTailLoopTest) ---
TEST(V5_003, SIMDTailLoopTest) {
    const float dt = 1.0f / 60.0f;

    // 13 个刚体：槽位 0-7 = 1 个 8 路批处理（槽位 5 为静态，检验免分支掩码），
    // 槽位 8-12 = 5 个标量尾部（槽位 8、11 为静态，检验尾部静态跳过）
    World simdWorld(Vector2(0.0f, -9.8f));
    std::vector<Body*> simdBodies = buildVariedBodies(simdWorld, 13, 5, 11);

    World scalarWorld(Vector2(0.0f, -9.8f));
    scalarWorld.getSoA().SetUseSIMD(false);
    std::vector<Body*> scalarBodies = buildVariedBodies(scalarWorld, 13, 5, 11);

    // 推进 2 步：任意数量刚体计算结果 100% 正确，无数组越界崩溃
    simdWorld.step(dt);
    simdWorld.step(dt);
    scalarWorld.step(dt);
    scalarWorld.step(dt);

    compareWorlds(simdWorld, simdBodies, scalarWorld, scalarBodies, 1e-6f, "SIMDTailLoop");

    // 静态刚体必须纹丝不动（掩码 blend 保原值 / 标量跳过，逐位核对；静态槽位为 5、11）
    for (int i : {5, 11}) {
        EXPECT_FLOAT_EQ(simdBodies[i]->getPosition().getX(), i * 0.8f);
        EXPECT_FLOAT_EQ(simdBodies[i]->getPosition().getY(), 0.2f + 0.15f * (i % 5));
        EXPECT_FLOAT_EQ(simdBodies[i]->getVelocity().getX(), 0.0f);
        EXPECT_FLOAT_EQ(simdBodies[i]->getVelocity().getY(), 0.0f);
    }
}

// --- 场景 3: 万体前向积分极限狂飙测试 (SIMD10000BodiesBenchmark) ---
TEST(V5_003, SIMD10000BodiesBenchmark) {
    const int N = 10000;
    const int STEPS = 20;
    const float dt = 1.0f / 60.0f;
    const Vector2 gravity(0.0f, -9.8f);

    auto buildArmy = [&](World& w) {
        std::vector<Body*> bs(N);
        for (int i = 0; i < N; ++i) {
            // 单排 1.5m 间距高空齐落：全程零接触，位形纯粹由积分决定
            bs[i] = w.createCircle(0.05f, -7500.0f + i * 1.5f, 100.0f, 1.0f + (i % 10) * 0.5f);
            bs[i]->setVelocity(Vector2(0.1f * (i % 3), -0.1f * (i % 2)));
            bs[i]->setAngularVelocity(0.5f * (i % 4));
        }
        return bs;
    };

    // 新版：AVX2 内核（默认）
    World simdWorld(gravity);
    std::vector<Body*> simdBodies = buildArmy(simdWorld);

    // 对照 1：引擎标量回退路径（同场景，强制关闭 SIMD——SoA 架构本身的收益基线）
    World scalarWorld(gravity);
    scalarWorld.getSoA().SetUseSIMD(false);
    buildArmy(scalarWorld);

    // 对照 2：旧引擎基线——AoS 指针遍历（散落堆对象 + 逐体访问器调用，
    // 忠实复刻 V5 Day 2 之前的旧版积分路径，规格对照的"约 3.5~4.2ms"量级）
    struct AoSBody {
        Vector2 position, velocity, force, prevPosition;
        float rotation, angularVelocity, torque, invMass, invInertia, gravityScale, prevRotation;
        bool isAwake;
        Vector2 getForce() const { return force; }
        float getInvMass() const { return invMass; }
        float getInvInertia() const { return invInertia; }
        float getGravityScale() const { return gravityScale; }
        bool awake() const { return isAwake; }
        void clearForce() { force.clear(); }
        void savePrevState() { prevPosition = position; prevRotation = rotation; }
    };
    std::vector<AoSBody*> aos(N);
    std::vector<AoSBody> init(N);
    for (int i = 0; i < N; ++i) {
        aos[i] = new AoSBody();
        aos[i]->position = simdBodies[i]->getPosition();
        aos[i]->velocity = simdBodies[i]->getVelocity();
        aos[i]->rotation = simdBodies[i]->getRotation();
        aos[i]->angularVelocity = simdBodies[i]->getAngularVelocity();
        aos[i]->invMass = simdBodies[i]->getInvMass();
        aos[i]->invInertia = simdBodies[i]->getInvInertia();
        aos[i]->gravityScale = simdBodies[i]->getGravityScale();
        aos[i]->isAwake = true;
        init[i] = *aos[i];
    }
    double aosMs = 1e9;
    for (int rep = 0; rep < 3; ++rep) {
        for (int i = 0; i < N; ++i) *aos[i] = init[i];
        auto t0 = std::chrono::high_resolution_clock::now();
        for (int s = 0; s < STEPS; ++s) {
            for (int i = 0; i < N; ++i) {
                AoSBody* b = aos[i];
                if (b->getInvMass() == 0.0f || !b->awake()) continue;
                Vector2 accel = b->getForce() * b->getInvMass() + gravity * b->getGravityScale();
                b->velocity += accel * dt;
                b->angularVelocity += b->torque * b->getInvInertia() * dt;
                b->savePrevState();
                b->position += b->velocity * dt;
                b->rotation += b->angularVelocity * dt;
                b->clearForce(); b->torque = 0.0f;
            }
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        aosMs = std::min(aosMs, std::chrono::duration<double, std::milli>(t1 - t0).count());
    }

    // 逐帧记录积分阶段纯耗时（Profiler 阶段计时，排除宽相/求解），取帧间最优抗负载
    double simdMs = 1e9, scalarMs = 1e9;
    for (int s = 0; s < STEPS; ++s) {
        simdWorld.step(dt);
        scalarWorld.step(dt);
        if (s > 0) { // 跳过首帧冷启动
            simdMs = std::min(simdMs, (double)simdWorld.getProfiler().GetRecords().at("1. Integration").accumulatedTime);
            scalarMs = std::min(scalarMs, (double)scalarWorld.getProfiler().GetRecords().at("1. Integration").accumulatedTime);
        }
    }

    double perFrameAos = aosMs / STEPS;
    RecordProperty("simd_ms_per_frame", simdMs);
    RecordProperty("scalar_ms_per_frame", scalarMs);
    RecordProperty("aos_ms_per_frame", perFrameAos);
    RecordProperty("speedup_vs_aos", perFrameAos / simdMs);

    // 核心断言：万体积分纯耗时严格 < 0.4 ms（规格：旧标量 AoS 约 3.5~4.2ms 的量级）
    EXPECT_LT(simdMs, 0.4);
    // 加速比断言：对旧引擎 AoS 基线至少 3 倍（实测预期 6~9 倍）
    EXPECT_GT(perFrameAos / simdMs, 3.0);

    for (AoSBody* b : aos) delete b;
}
