// GoogleTest 版 V5/004：内存与缓存基准评估——V5 第一阶段收官大考
// 用例 1 分配器吞吐对决：10 万次 16B~1024B 随机分配回收，BlockAllocator
//   总耗时必须低于原生 malloc/free 的 35%
// 用例 2 缓存友好度能效阶梯：10,000 刚体，纯 SoA 标量 vs AoS 指针遍历
//   耗时下降 > 30%（纯粹依靠缓存行连续性取得的战果）
// 用例 3 AVX2 矢量爆发终极加速比：SoA AVX2 vs AoS 标量，
//   单步积分纯耗时 < 0.40ms 且加速比 ≥ 6.0 倍
// 用例 4 CSV 报表导出与数据完整性：四配置 × 三档规模（500/2000/10000）
//   完整数据矩阵写入 output/benchmark_phase1.csv；四配置推进 100 步后的
//   位置/速度校验和误差严格 < 1e-5（极速优化零精度牺牲）
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "SoABody.h"
#include "Circle.h"
#include "Setting.h"
#include "BlockAllocator.h"
#include "BenchmarkTool.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <set>
#include <string>
#include <vector>

// 确定性随机数（xorshift32）：全测试可复现
static uint32_t g_state = 0x12345678u;
static uint32_t xr()
{
    g_state ^= g_state << 13;
    g_state ^= g_state >> 17;
    g_state ^= g_state << 5;
    return g_state;
}

// 防死代码消除（Anti-DCE）：volatile 校验和吸收槽，强迫编译器保留所有计算指令
static volatile float g_dummySink = 0.0f;

namespace {

// ================= 旧引擎 AoS 复刻（配置 A/B 共用） =================
// 忠实复刻 V4 原始版积分路径：散落堆对象 + 逐体访问器调用 + 直接成员写
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

inline void aosStep(AoSBody* b, float dt, const Vector2& gravity)
{
    if (b->getInvMass() == 0.0f || !b->awake()) return;
    Vector2 accel = b->getForce() * b->getInvMass() + gravity * b->getGravityScale();
    b->velocity += accel * dt;
    b->angularVelocity += b->torque * b->getInvInertia() * dt;
    b->savePrevState();
    b->position += b->velocity * dt;
    b->rotation += b->angularVelocity * dt;
    b->clearForce(); b->torque = 0.0f;
}

// ================= 军队构建与快照 =================
std::vector<Body*> buildWorldArmy(World& w, int N)
{
    std::vector<Body*> bs(N);
    for (int i = 0; i < N; ++i) {
        // 单排 1.5m 间距高空齐落：全程零接触，位形纯粹由积分决定
        bs[i] = w.createCircle(0.05f, -7500.0f + i * 1.5f, 100.0f, 1.0f + (i % 10) * 0.5f);
        bs[i]->setVelocity(Vector2(0.1f * (i % 3), -0.1f * (i % 2)));
        bs[i]->setAngularVelocity(0.5f * (i % 4));
    }
    return bs;
}

std::vector<AoSBody*> snapshotAoS(const std::vector<Body*>& bs)
{
    std::vector<AoSBody*> aos(bs.size());
    for (size_t i = 0; i < bs.size(); ++i) {
        aos[i] = new AoSBody();
        aos[i]->position = bs[i]->getPosition();
        aos[i]->velocity = bs[i]->getVelocity();
        aos[i]->rotation = bs[i]->getRotation();
        aos[i]->angularVelocity = bs[i]->getAngularVelocity();
        aos[i]->invMass = bs[i]->getInvMass();
        aos[i]->invInertia = bs[i]->getInvInertia();
        aos[i]->gravityScale = bs[i]->getGravityScale();
        aos[i]->isAwake = true;
    }
    return aos;
}

// ================= 校验和（位置与速度哈希） =================
void checksumAoS(const std::vector<AoSBody*>& aos, double& posSum, double& velSum)
{
    posSum = 0.0; velSum = 0.0;
    for (AoSBody* b : aos) {
        posSum += (double)b->position.getX() + (double)b->position.getY();
        velSum += (double)b->velocity.getX() + (double)b->velocity.getY();
    }
}

void checksumSoA(BodySoA& soa, double& posSum, double& velSum)
{
    posSum = 0.0; velSum = 0.0;
    const int n = soa.GetCount();
    for (int i = 0; i < n; ++i) {
        posSum += (double)soa.positionsX[i] + (double)soa.positionsY[i];
        velSum += (double)soa.velocitiesX[i] + (double)soa.velocitiesY[i];
    }
}

// 相对误差（防除零）
double relDiff(double a, double b)
{
    double denom = std::abs(a) > 1e-9 ? std::abs(a) : 1.0;
    return std::abs(a - b) / denom;
}

} // namespace

// --- 用例 1: 分配器吞吐对比大胜 (AllocatorThroughputBenchmark) ---
TEST(V5_004, AllocatorThroughputBenchmark) {
    const int OPS = 100000; // 十万次 16B~1024B 随机小内存申请与释放
    static int sizes[OPS];
    for (int i = 0; i < OPS; ++i) sizes[i] = 16 + (int)(xr() % 1009);

    Physics2D::BenchmarkSuite suite("V5_Phase1_Allocator");

    auto mallocTask = [&]() {
        void* p = nullptr;
        for (int i = 0; i < OPS; ++i) {
            p = std::malloc((size_t)sizes[i]);
            std::free(p);
        }
        g_dummySink += (p != nullptr) ? 0.0f : 0.0f; // Anti-DCE
    };
    auto poolTask = [&]() {
        BlockAllocator pool;
        void* p = nullptr;
        for (int i = 0; i < OPS; ++i) {
            p = pool.Allocate(sizes[i]);
            pool.Free(p, sizes[i]);
        }
        g_dummySink += (p != nullptr) ? 0.0f : 0.0f;
    };

    Physics2D::BenchmarkResult rMalloc = suite.RunBenchmark("A_malloc_free", OPS, 3, 5, mallocTask);
    Physics2D::BenchmarkResult rPool   = suite.RunBenchmark("B_BlockAllocator", OPS, 3, 5, poolTask);

    RecordProperty("malloc_median_ms", rMalloc.medianMs);
    RecordProperty("pool_median_ms", rPool.medianMs);

    // 核心断言：BlockAllocator 总耗时严格低于原生 malloc 的 35%
    EXPECT_LT(rPool.medianMs, rMalloc.medianMs * 0.35);
}

// --- 用例 2: 缓存友好度能效阶梯测试 (CacheEfficiencyStepTest) ---
TEST(V5_004, CacheEfficiencyStepTest) {
    const int N = 10000;
    const float dt = 1.0f / 60.0f;
    const Vector2 gravity(0.0f, -9.8f);

    World world(gravity);
    std::vector<Body*> bs = buildWorldArmy(world, N);
    std::vector<AoSBody*> aos = snapshotAoS(bs);

    Physics2D::BenchmarkSuite suite("V5_Phase1_Integration");

    // 配置 A（V4 原始版）：AoS 指针遍历 + 标量计算
    auto taskA = [&]() {
        for (int i = 0; i < N; ++i) aosStep(aos[i], dt, gravity);
        g_dummySink += aos[0]->position.getX();
    };
    // 配置 C（SoA 纯连续版）：SoA 扁平数组 + 标量循环（引擎回退内核）
    auto taskC = [&]() {
        world.getSoA().IntegrateScalar(dt, gravity);
        g_dummySink += world.getSoA().positionsX[0];
    };

    Physics2D::BenchmarkResult rA = suite.RunBenchmark("A_AoS_Scalar", N, 3, 7, taskA);
    Physics2D::BenchmarkResult rC = suite.RunBenchmark("C_SoA_Scalar", N, 3, 7, taskC);

    RecordProperty("aos_median_ms", rA.medianMs);
    RecordProperty("soa_scalar_median_ms", rC.medianMs);

    // 核心断言：纯 SoA 相比旧版 AoS 耗时下降 > 30%（缓存行连续性的纯粹战果）
    EXPECT_LT(rC.medianMs, rA.medianMs * 0.70);

    for (AoSBody* b : aos) delete b;
}

// --- 用例 3: AVX2 矢量爆发终极加速比测试 (AVX2SpeedupBenchmark) ---
TEST(V5_004, AVX2SpeedupBenchmark) {
    const int N = 10000;
    const float dt = 1.0f / 60.0f;
    const Vector2 gravity(0.0f, -9.8f);

    World world(gravity);
    std::vector<Body*> bs = buildWorldArmy(world, N);
    std::vector<AoSBody*> aos = snapshotAoS(bs);

    Physics2D::BenchmarkSuite suite("V5_Phase1_Integration");

    // 配置 A：AoS 标量（V4 基线）
    auto taskA = [&]() {
        for (int i = 0; i < N; ++i) aosStep(aos[i], dt, gravity);
        g_dummySink += aos[0]->position.getX();
    };
    // 配置 D（终极完全体）：SoA 扁平数组 + AVX2 8 路 SIMD
    auto taskD = [&]() {
        world.getSoA().IntegrateSIMD(dt, gravity);
        g_dummySink += world.getSoA().positionsX[0];
    };

    Physics2D::BenchmarkResult rA = suite.RunBenchmark("A_AoS_Scalar", N, 3, 7, taskA);
    Physics2D::BenchmarkResult rD = suite.RunBenchmark("D_SoA_AVX2", N, 3, 7, taskD);

    double speedup = rA.medianMs / rD.medianMs;
    RecordProperty("aos_median_ms", rA.medianMs);
    RecordProperty("simd_median_ms", rD.medianMs);
    RecordProperty("avx2_speedup", speedup);

    // 核心断言：单步积分纯耗时 < 0.40ms；相比 V4 基线加速比 ≥ 6.0 倍
    EXPECT_LT(rD.medianMs, 0.40);
    EXPECT_GE(speedup, 6.0);

    for (AoSBody* b : aos) delete b;
}

// --- 用例 4: CSV 报表导出与数据完整性断言 (BenchmarkReportExport) ---
TEST(V5_004, BenchmarkReportExport) {
    const int scales[] = { 500, 2000, 10000 };
    const float dt = 1.0f / 60.0f;
    const Vector2 gravity(0.0f, -9.8f);

    Physics2D::BenchmarkSuite suite("V5_Phase1");

    for (int S : scales) {
        // ---- 军队与四种配置的载体 ----
        World world(gravity);                       // 配置 C 载体（标量内核）
        std::vector<Body*> bs = buildWorldArmy(world, S);
        std::vector<AoSBody*> aosRef = snapshotAoS(bs);            // 配置 A：散布 new
        BlockAllocator pool;
        std::vector<AoSBody*> aosPool(S);                          // 配置 B：内存池分配
        for (int i = 0; i < S; ++i) {
            aosPool[i] = (AoSBody*)pool.Allocate((int)sizeof(AoSBody));
            *aosPool[i] = *aosRef[i];
        }
        World worldD(gravity);                       // 配置 D 载体（AVX2 内核）
        buildWorldArmy(worldD, S);

        // ---- 校验和回归：四配置各推进 100 步，比对位置与速度校验和 ----
        double aPos, aVel, bPos, bVel, cPos, cVel, dPos, dVel;
        auto run100A = [&](std::vector<AoSBody*>& army) {
            for (int s = 0; s < 100; ++s)
                for (int i = 0; i < S; ++i) aosStep(army[i], dt, gravity);
        };
        run100A(aosRef);  checksumAoS(aosRef, aPos, aVel);
        run100A(aosPool); checksumAoS(aosPool, bPos, bVel);
        for (int s = 0; s < 100; ++s) world.getSoA().IntegrateScalar(dt, gravity);
        checksumSoA(world.getSoA(), cPos, cVel);
        for (int s = 0; s < 100; ++s) worldD.getSoA().IntegrateSIMD(dt, gravity);
        checksumSoA(worldD.getSoA(), dPos, dVel);

        // 四组配置两两校验和相对误差严格 < 1e-5（A/B/C 为同标量序列，误差应为 0；
        // D 走 FMA 内核，存在 ~ulp 级舍入差异，仍在 1e-5 关口之内）
        EXPECT_LT(relDiff(aPos, bPos), 1e-5) << "S=" << S << " A vs B pos";
        EXPECT_LT(relDiff(aVel, bVel), 1e-5) << "S=" << S << " A vs B vel";
        EXPECT_LT(relDiff(aPos, cPos), 1e-5) << "S=" << S << " A vs C pos";
        EXPECT_LT(relDiff(aVel, cVel), 1e-5) << "S=" << S << " A vs C vel";
        EXPECT_LT(relDiff(aPos, dPos), 1e-5) << "S=" << S << " A vs D pos";
        EXPECT_LT(relDiff(aVel, dVel), 1e-5) << "S=" << S << " A vs D vel";
        RecordProperty(("relErr_AD_pos_S" + std::to_string(S)).c_str(), relDiff(aPos, dPos));

        // ---- 计时矩阵（预热 3 + 采样 5，中位数为主判据） ----
        suite.RunBenchmark("A_AoS_Scatter", S, 3, 5, [&]() {
            for (int i = 0; i < S; ++i) aosStep(aosRef[i], dt, gravity);
            g_dummySink += aosRef[0]->position.getX();
        });
        suite.RunBenchmark("B_AoS_BlockAllocator", S, 3, 5, [&]() {
            for (int i = 0; i < S; ++i) aosStep(aosPool[i], dt, gravity);
            g_dummySink += aosPool[0]->position.getX();
        });
        suite.RunBenchmark("C_SoA_Scalar", S, 3, 5, [&]() {
            world.getSoA().IntegrateScalar(dt, gravity);
            g_dummySink += world.getSoA().positionsX[0];
        });
        suite.RunBenchmark("D_SoA_AVX2", S, 3, 5, [&]() {
            worldD.getSoA().IntegrateSIMD(dt, gravity);
            g_dummySink += worldD.getSoA().positionsX[0];
        });

        for (AoSBody* b : aosRef) delete b;
        // aosPool 块由 pool 析构 Clear() 兜底回收
    }

    // ---- 导出 CSV 报表 ----
    const std::string csvPath = "output/benchmark_phase1.csv";
    suite.ExportToCSV(csvPath);

    // ---- 数据完整性断言：文件存在 + 表头 + 4 配置 × 3 档规模完整矩阵 ----
    std::ifstream file(csvPath);
    ASSERT_TRUE(file.good()) << "CSV 报表必须成功生成";

    std::string header;
    std::getline(file, header);
    EXPECT_EQ(header, "Suite,Case,ObjectCount,MeanMs,MedianMs,MinMs,MaxMs,StdDevMs");

    std::set<std::string> combos;
    std::string line;
    int dataRows = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        // Case,ObjectCount 两列组成完整性键
        size_t c1 = line.find(',');
        size_t c2 = line.find(',', c1 + 1);
        size_t c3 = line.find(',', c2 + 1);
        if (c1 == std::string::npos || c2 == std::string::npos) continue;
        combos.insert(line.substr(c1 + 1, c2 - c1 - 1) + "@" + line.substr(c2 + 1, c3 - c2 - 1));
        ++dataRows;
    }
    EXPECT_EQ(dataRows, 12) << "必须恰好 4 配置 × 3 档规模 = 12 行数据";
    for (const char* cfg : { "A_AoS_Scatter", "B_AoS_BlockAllocator", "C_SoA_Scalar", "D_SoA_AVX2" }) {
        for (int s : scales) {
            EXPECT_TRUE(combos.count(std::string(cfg) + "@" + std::to_string(s)) == 1)
                << "缺少 " << cfg << " @ " << s;
        }
    }
}
