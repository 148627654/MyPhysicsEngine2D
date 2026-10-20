// GoogleTest 版 V5/002：刚体数据导向设计 (SoA) 与缓存行对齐改造验收
// 场景 1 SoA 增删置换与索引映射：10 刚体槽位 0~9 → 删除槽位 3 →
//   末尾刚体 Swap-and-Pop 精准搬家到槽位 3，句柄自动同步，数据无一错位
// 场景 2 数据双向一致性：setPosition/setVelocity 等 API 无缝穿透读写 SoA 槽位；
//   1 步积分后 getPosition() 与 SoA 运算值一致（外部业务代码 0 感知）
// 场景 3 5000 刚体积分性能对照：SoA 流水线 vs 旧版 AoS 指针遍历——
//   数值结果一致（Day 3 起引擎走 AVX2 FMA 内核，舍入差异 ~ulp 级按容差验收），
//   积分耗时下降 35% 以上（< 1.3ms/帧）
// 注：Day 3 后 SoA 数组升级为拆分单精度流（positionsX/Y 等），本文件
//   的槽位断言统一走 BodySoA 的 Get/Set 便捷接口
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "SoABody.h"
#include "Circle.h"
#include "Setting.h"
#include <chrono>
#include <cmath>
#include <vector>

// 逐位相等断言助手（存储型写穿断言用：无算术运算的存取必须逐位）
static bool vecEq(const Vector2& a, const Vector2& b)
{
    return a.getX() == b.getX() && a.getY() == b.getY();
}

// --- 场景 1: SoA 增删置换与索引映射正确性测试 (SoAAllocationAndSwapTest) ---
TEST(V5_002, SoAAllocationAndSwapTest) {
    World world(Vector2(0.0f, -9.8f));

    // 连续创建 10 个静态刚体（不参与积分，位置数据在置换后可逐位核对）
    std::vector<Body*> bodies;
    for (int i = 0; i < 10; ++i) {
        bodies.push_back(world.createCircle(0.2f, i * 1.0f, 10.0f, 0.0f));
    }

    // 槽位索引严格为 0 ~ 9（创建顺序）
    for (int i = 0; i < 10; ++i) {
        EXPECT_EQ(bodies[i]->getSoaIndex(), i);
    }
    EXPECT_EQ(world.getSoA().GetCount(), 10);

    // 给槽位 9 的刚体打上独特标记，作为 Swap-and-Pop 的数据完整性探针
    bodies[9]->setPosition(Vector2(99.0f, 88.0f));
    EXPECT_TRUE(vecEq(world.getSoA().GetPosition(9), Vector2(99.0f, 88.0f)));

    // 删除索引为 3 的刚体（延迟销毁，step 开头真正移除）
    Body* body3 = bodies[3];
    world.destroyBody(body3);
    EXPECT_EQ(world.getSoA().GetCount(), 10); // 尚未 flush，槽位还在
    world.step(1.0f / 60.0f);

    // 核心断言：总数变为 9；原索引 9 的刚体被精准移至槽位 3，句柄自动同步为 3
    // （被删刚体已随 step 真正释放，其句柄脱离由 FreeSlot 在 delete 前完成，不可读已释放对象）
    EXPECT_EQ(world.getSoA().GetCount(), 9);
    EXPECT_EQ(world.getSoA().GetOwner(3), bodies[9]);
    EXPECT_EQ(bodies[9]->getSoaIndex(), 3);

    // 数据无一错位：槽位 3 承载的是原槽位 9 的完整数据（静态刚体全程不积分，可逐位核对）
    EXPECT_TRUE(vecEq(world.getSoA().GetPosition(3), Vector2(99.0f, 88.0f)));
    EXPECT_TRUE(vecEq(world.getSoA().GetVelocity(3), Vector2(0.0f, 0.0f)));
    EXPECT_EQ(world.getSoA().GetRotation(3), bodies[9]->getRotation());
    EXPECT_EQ(world.getSoA().GetInvMass(3), bodies[9]->getInvMass());

    // 其余槽位原地不动
    for (int i = 0; i < 3; ++i)  EXPECT_EQ(world.getSoA().GetOwner(i), bodies[i]);
    for (int i = 4; i < 9; ++i)  EXPECT_EQ(world.getSoA().GetOwner(i), bodies[i]);
}

// --- 场景 2: 数据双向一致性测试 (BidirectionalSyncTest) ---
TEST(V5_002, BidirectionalSyncTest) {
    World world(Vector2(0.0f, -9.8f));
    Body* ball = world.createCircle(0.5f, 0.0f, 5.0f, 1.0f);
    int idx = ball->getSoaIndex();
    EXPECT_EQ(idx, 0);

    BodySoA& soa = world.getSoA();

    // 挂载同步：构造参数已镜像进 SoA 槽位
    EXPECT_TRUE(vecEq(soa.GetPosition(idx), Vector2(0.0f, 5.0f)));
    EXPECT_TRUE(vecEq(soa.GetVelocity(idx), Vector2(0.0f, 0.0f)));
    EXPECT_EQ(soa.GetInvMass(idx), ball->getInvMass());
    EXPECT_EQ(soa.GetInvInertia(idx), ball->getInvInertia());
    EXPECT_EQ(soa.GetGravityScale(idx), 1.0f);

    // 写入穿透：API 修改立刻反映到 SoA 槽位（纯存储，逐位断言）
    ball->setPosition(Vector2(10.0f, 20.0f));
    EXPECT_TRUE(vecEq(soa.GetPosition(idx), Vector2(10.0f, 20.0f)));
    ball->setVelocity(Vector2(1.0f, 2.0f));
    EXPECT_TRUE(vecEq(soa.GetVelocity(idx), Vector2(1.0f, 2.0f)));
    ball->setGravityScale(2.0f);
    EXPECT_EQ(soa.GetGravityScale(idx), 2.0f);
    ball->setAngularVelocity(0.5f);
    EXPECT_EQ(soa.GetAngularVel(idx), 0.5f);
    ball->addTorque(3.0f);
    EXPECT_EQ(soa.GetTorque(idx), 3.0f);

    // 手工按标量公式计算 1 步积分期望值
    const float dt = 1.0f / 60.0f;
    Vector2 expectV = soa.GetVelocity(idx) +
        (soa.GetForce(idx) * soa.GetInvMass(idx) + Vector2(0.0f, -9.8f) * soa.GetGravityScale(idx)) * dt;
    Vector2 expectP = soa.GetPosition(idx) + expectV * dt;
    float expectR = soa.GetRotation(idx) + (soa.GetAngularVel(idx) + soa.GetTorque(idx) * soa.GetInvInertia(idx) * dt) * dt;

    // 模拟 1 步积分推进
    world.step(dt);

    // 读取穿透：getPosition() 与 SoA 数组、与手工期望一致。
    // Day 3 起引擎积分走 AVX2 FMA 内核，与标量手工期望存在 ~ulp 级舍入
    // 差异（浮点语义等价而非逐位相等），按 1e-5/1e-6 容差验收
    EXPECT_NEAR(ball->getPosition().getX(), soa.GetPosition(idx).getX(), 1e-5f);
    EXPECT_NEAR(ball->getPosition().getY(), soa.GetPosition(idx).getY(), 1e-5f);
    EXPECT_NEAR(ball->getPosition().getX(), expectP.getX(), 1e-5f);
    EXPECT_NEAR(ball->getPosition().getY(), expectP.getY(), 1e-5f);
    EXPECT_NEAR(ball->getVelocity().getX(), expectV.getX(), 1e-6f);
    EXPECT_NEAR(ball->getVelocity().getY(), expectV.getY(), 1e-6f);
    EXPECT_NEAR(ball->getVelocity().getX(), soa.GetVelocity(idx).getX(), 1e-6f);
    EXPECT_NEAR(ball->getVelocity().getY(), soa.GetVelocity(idx).getY(), 1e-6f);
    EXPECT_NEAR(ball->getRotation(), expectR, 1e-6f);
    EXPECT_NEAR(ball->getRotation(), soa.GetRotation(idx), 1e-6f);
    // 积分后力/扭矩按旧语义清零（纯存储，逐位断言）
    EXPECT_TRUE(vecEq(soa.GetForce(idx), Vector2(0.0f, 0.0f)));
    EXPECT_EQ(soa.GetTorque(idx), 0.0f);
}

// --- 场景 3: 5000 刚体积分性能对照大考 (SoAVsAoSBenchmark) ---
TEST(V5_002, SoAVsAoSBenchmark) {
    const int N = 5000;
    const int STEPS = 60;
    const float dt = 1.0f / 60.0f;
    const Vector2 gravity(0.0f, -9.8f);

    // ---------- 新版：World + SoA（Day 3 起为 AVX2 FMA 内核） ----------
    World world(gravity);
    std::vector<Body*> bodies(N);
    for (int i = 0; i < N; ++i) {
        // 单排 1.5m 间距高空齐落：全程零接触，位形纯粹由积分决定
        bodies[i] = world.createCircle(0.05f, -3750.0f + i * 1.5f, 100.0f, 1.0f + (i % 10) * 0.5f);
        bodies[i]->setVelocity(Vector2(0.1f * (i % 3), -0.1f * (i % 2)));
        bodies[i]->setAngularVelocity(0.5f * (i % 4));
    }

    // ---------- 旧版参考：AoS 指针遍历（散落堆对象 + 逐体访问器调用，忠实复刻旧引擎路径） ----------
    // 旧版积分循环原样：getInvMass/isAwake/getForce/getGravityScale/getInvInertia 5 次
    // 访问器调用 + savePrevState/clearForce 2 次调用，速度/位置/角度为直接成员写
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
    std::vector<AoSBody> init(N); // 初态快照（每轮计时前复位）
    for (int i = 0; i < N; ++i) {
        aos[i] = new AoSBody();
        aos[i]->position = bodies[i]->getPosition();
        aos[i]->velocity = bodies[i]->getVelocity();
        aos[i]->rotation = bodies[i]->getRotation();
        aos[i]->angularVelocity = bodies[i]->getAngularVelocity();
        aos[i]->invMass = bodies[i]->getInvMass();
        aos[i]->invInertia = bodies[i]->getInvInertia();
        aos[i]->gravityScale = bodies[i]->getGravityScale();
        aos[i]->isAwake = true;
        init[i] = *aos[i];
    }

    // 旧版 AoS 积分计时（3 轮取最优，抗机器负载波动；复位不计时）
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

    // 新版跑 World：积分阶段耗时从 Profiler 逐帧累加（真实引擎流水线）
    double soaMs = 0.0;
    for (int s = 0; s < STEPS; ++s) {
        world.step(dt);
        soaMs += world.getProfiler().GetRecords().at("1. Integration").accumulatedTime;
    }

    // 数值一致性：逐体位姿/速度/角度与旧版参考一致。
    // Day 3 起引擎走 FMA 内核，与标量参考存在 ~ulp 级舍入差异，60 帧累计后
    // 按量级给容差：位置 ~100 量级 → 1e-3，速度 ~10 → 1e-4，角度 ~1 → 1e-5
    bool ok = true;
    for (int i = 0; i < N && ok; ++i) {
        AoSBody* b = aos[i];
        if (std::abs(bodies[i]->getPosition().getX() - b->position.getX()) > 1e-3f ||
            std::abs(bodies[i]->getPosition().getY() - b->position.getY()) > 1e-3f) ok = false;
        if (std::abs(bodies[i]->getVelocity().getX() - b->velocity.getX()) > 1e-4f ||
            std::abs(bodies[i]->getVelocity().getY() - b->velocity.getY()) > 1e-4f) ok = false;
        if (std::abs(bodies[i]->getRotation() - b->rotation) > 1e-5f) ok = false;
        if (std::abs(bodies[i]->getAngularVelocity() - b->angularVelocity) > 1e-4f) ok = false;
    }
    EXPECT_TRUE(ok) << "SoA(SIMD) 与 AoS 参考积分结果必须数值一致";

    double perFrameAos = aosMs / STEPS;
    double perFrameSoa = soaMs / STEPS;
    RecordProperty("aos_ms_per_frame", perFrameAos);
    RecordProperty("soa_ms_per_frame", perFrameSoa);

    // 性能断言：积分阶段纯耗时下降 35% 以上，且绝对耗时 < 1.3ms/帧
    EXPECT_LT(perFrameSoa, perFrameAos * 0.65);
    EXPECT_LT(perFrameSoa, 1.3);

    for (AoSBody* b : aos) delete b;
}
