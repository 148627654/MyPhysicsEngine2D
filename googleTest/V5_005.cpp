// GoogleTest 版 V5/005：低延迟工作窃取线程池验收
// 用例 1 基础并发与无死锁：1 万个浮点微任务并发执行，结果 100% 精确无偏差
// 用例 2 工作窃取机制有效性：1000 个重任务全部塞给 0 号线程，
//   其余核心窃取成功计数均显著 > 0（负载自动均衡摊平）
// 用例 3 微秒级极速调度延迟压测：10 万极小任务，单任务平均调度分发
//   开销严格 < 2.0 μs（无全局锁架构的极致低延迟）
// 用例 4 主线程协作等待：ParallelFor 等待期间主线程主动窃取执行，
//   主线程执行计数 > 0（零休眠调度浪费）
#include <gtest/gtest.h>
#include "ThreadPool.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

namespace {

// 确定性微负载：真实睡眠（模拟重计算任务，OS 调度粒度 ~毫秒内）
inline void busyWorkUs(int us)
{
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

} // namespace

// --- 用例 1: 基础并发与无死锁测试 (ThreadPoolBasicTest) ---
TEST(V5_005, ThreadPoolBasicTest) {
    Physics2D::ThreadPool pool; // 硬件核心数

    const int N = 10000;
    std::atomic<int> counter(N);
    // 【浮点累加教训】每个加数 i×0.125 本身精确可表示，但部分和超过 2^22 后
    // float 的 ulp = 0.5（非 0.125）——1 万次累加在 float 中必然产生 ~263 的
    // 舍入偏差（单线程实测 6,249,112 vs 6,249,375，与并发/原子实现无关）。
    // 改为互斥锁保护的 double 累加：6.25M 量级 ≪ 2^53，全程精确、顺序无关
    double sum = 0.0;
    std::mutex sumMutex;

    // 每个任务：f = i × 0.125，附带一组无副作用浮点运算负载（结果仍精确为 f）
    for (int i = 0; i < N; ++i) {
        pool.Submit([i, &counter, &sum, &sumMutex]() {
            const float f = (float)i * 0.125f;
            const float load = f * 3.0f - f * 2.0f; // 浮点运算负载
            {
                std::lock_guard<std::mutex> lock(sumMutex);
                sum += (double)load;
            }
            counter.fetch_sub(1, std::memory_order_release);
        });
    }

    pool.Wait(counter); // 协作等待：主线程也参与偷任务

    // 断言：所有任务 100% 准确执行完毕，无死锁停滞，累加零偏差
    EXPECT_EQ(counter.load(), 0);
    EXPECT_EQ(pool.GetExecutedCount(), (uint64_t)N);
    EXPECT_DOUBLE_EQ(sum, 6249375.0); // Σ i·0.125 = 6,249,375 精确
}

// --- 用例 2: 工作窃取机制有效性测试 (WorkStealingValidationTest) ---
TEST(V5_005, WorkStealingValidationTest) {
    Physics2D::ThreadPool pool;
    const int W = pool.GetNumThreads();
    if (W < 2) {
        GTEST_SKIP() << "单核环境无法验证跨核工作窃取";
        return;
    }

    // 故意只向 0 号工作线程塞入 1000 个重任务，其余队列全空
    const int N = 1000;
    std::atomic<int> counter(N);
    for (int i = 0; i < N; ++i) {
        pool.SubmitTo(0, [&counter]() {
            busyWorkUs(150); // 重任务
            counter.fetch_sub(1, std::memory_order_release);
        });
    }

    pool.Wait(counter);
    EXPECT_EQ(counter.load(), 0);

    // 断言：其余核心的窃取成功计数均显著大于 0——空闲核心主动帮 0 号分担
    uint64_t totalSteals = 0;
    for (int w = 1; w < W; ++w) {
        const uint64_t s = pool.GetStealCount(w);
        EXPECT_GT(s, 0u) << "工作线程 " << w << " 应窃取到任务";
        totalSteals += s;
    }
    EXPECT_GT(totalSteals, 0u);
    EXPECT_GT(pool.GetStealCount(), 0u);
    RecordProperty("total_steals", totalSteals);
}

// --- 用例 3: 微秒级极速调度延迟压测 (SchedulerLatencyBenchmark) ---
TEST(V5_005, SchedulerLatencyBenchmark) {
    Physics2D::ThreadPool pool;

    const int N = 100000; // 十万个极小任务
    std::atomic<int> counter(N);
    std::atomic<int> sink(0);

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; ++i) {
        pool.Submit([&counter, &sink]() {
            sink.fetch_add(1, std::memory_order_relaxed); // 极小任务：一次原子累加
            counter.fetch_sub(1, std::memory_order_release);
        });
    }
    pool.Wait(counter);
    auto t1 = std::chrono::high_resolution_clock::now();

    const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double perTaskUs = totalMs * 1000.0 / (double)N;

    RecordProperty("total_ms", totalMs);
    RecordProperty("avg_us_per_task", perTaskUs);
    RecordProperty("throughput_tasks_per_sec", N / (totalMs / 1000.0));

    // 断言：全部执行 + 单任务平均调度分发开销严格 < 2.0 μs
    EXPECT_EQ(counter.load(), 0);
    EXPECT_EQ(sink.load(), N);
    EXPECT_LT(perTaskUs, 2.0);
}

// --- 用例 4: 主线程协作等待断言 (CooperativeWaitTest) ---
TEST(V5_005, CooperativeWaitTest) {
    Physics2D::ThreadPool pool;

    // 在主线程中启动 ParallelFor 并挂起等待：每个元素为重负载，
    // 给主线程留足协作窃取的窗口
    const int N = 64;
    pool.ParallelFor(0, N, [](int) {
        busyWorkUs(200);
    });

    // 断言：主线程执行任务计数器 > 0——等待期间没有摸鱼休眠，而是主动协助
    EXPECT_GT(pool.GetMainExecutedCount(), 0u);
    // 执行数 = 批次数 = min(N, 每核 4 批)；主线程协作执行计入 m_mainExecuted（独立统计）
    const uint64_t expectedBatches =
        (uint64_t)((N < pool.GetNumThreads() * 4) ? N : pool.GetNumThreads() * 4);
    EXPECT_EQ(pool.GetExecutedCount() + pool.GetMainExecutedCount(), expectedBatches);
    RecordProperty("main_executed", pool.GetMainExecutedCount());
}
