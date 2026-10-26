#pragma once
// ThreadPool.h — 低延迟工作窃取线程池（V5 Day 5）
//
// 专为实时物理引擎定制：每核独立双端队列 + 跨核随机工作窃取，
// 消灭传统"全局队列 + 全局锁"的多核锁竞争与伪共享风暴；
// 主线程协作式等待（等待期间化身 Worker 偷任务干活，0 休眠调度浪费）。
#include "WorkStealingQueue.h"
#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

namespace Physics2D {

class ThreadPool {
public:
    /// @param numThreads 工作线程数；<=0 时取硬件最大并发数（至少 1）
    explicit ThreadPool(int numThreads = 0);
    ~ThreadPool();

    /// 确保所有待办任务完全消费后平稳 join（幂等）
    void Shutdown();

    inline int GetNumThreads() const { return m_numThreads; }

    /// 轮询分发任务到下一个工作线程（负载均衡）
    void Submit(const std::function<void()>& task);
    /// 定向投递到指定工作线程的本地队列（队列满自旋重试）
    void SubmitTo(int workerId, const std::function<void()>& task);

    /// 协作式等待：主线程在计数器归零前不断窃取并执行任务（不摸鱼、不休眠）
    void Wait(std::atomic<int>& counter);

    /// 通用并行循环：把 [start, end) 切分为若干子批次分派各核并发执行，
    /// 主线程协作参与计算，返回时全部完成
    void ParallelFor(int start, int end, const std::function<void(int)>& func);

    // ---- 验收统计 ----
    uint64_t GetExecutedCount() const { return m_executedCount.load(std::memory_order_relaxed); }
    uint64_t GetStealCount() const { return m_stealCount.load(std::memory_order_relaxed); }
    uint64_t GetStealCount(int workerId) const { return m_stealCounts[workerId].load(std::memory_order_relaxed); }
    uint64_t GetMainExecutedCount() const { return m_mainExecuted.load(std::memory_order_relaxed); }
    /// 每工作线程累计忙时（任务执行耗时，纳秒）——负载分布诊断（岛屿并行验收用）
    uint64_t GetWorkerBusyNs(int workerId) const { return m_workerBusyNs[workerId].load(std::memory_order_relaxed); }
    uint64_t GetMainBusyNs() const { return m_mainBusyNs.load(std::memory_order_relaxed); }

private:
    using Task = std::function<void()>;

    void WorkerLoop(int workerId);
    // 随机起点轮询窃取（防扎堆）；workerId < 0 表示主线程（无本地队列，全部可偷）
    bool TrySteal(int workerId, Task& task);
    bool AllQueuesEmpty() const;

    int m_numThreads;
    std::vector<std::thread> m_threads;
    // 队列/计数器含 std::atomic 成员（不可移动），用 unique_ptr 数组原地构造
    std::unique_ptr<WorkStealingQueue[]> m_queues;
    std::unique_ptr<std::atomic<uint64_t>[]> m_stealCounts;
    std::unique_ptr<std::atomic<uint64_t>[]> m_workerBusyNs;

    std::atomic<bool> m_shutdown{false};
    std::atomic<uint32_t> m_nextWorker{0};   // Submit 轮询游标
    std::atomic<uint64_t> m_executedCount{0};
    std::atomic<uint64_t> m_stealCount{0};   // 全局窃取成功计数
    std::atomic<uint64_t> m_mainExecuted{0}; // 主线程协作执行计数
    std::atomic<uint64_t> m_mainBusyNs{0};   // 主线程协作忙时
};

} // namespace Physics2D
