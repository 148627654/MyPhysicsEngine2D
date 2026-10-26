#include "ThreadPool.h"
#include <immintrin.h> // _mm_pause

namespace Physics2D {

namespace {
// 每线程独立随机数状态：WorkerLoop/Wait 用线程身份混合种子（0x85EBCA6B 黄金比），
// 各线程窃取起点序列互不相同——防空闲线程扎堆偷同一个受害者
thread_local uint32_t t_rngState = 0x9E3779B9u;
uint32_t nextRand()
{
    uint32_t s = t_rngState;
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    t_rngState = s;
    return s;
}
} // namespace

ThreadPool::ThreadPool(int numThreads)
{
    if (numThreads <= 0) {
        numThreads = (int)std::thread::hardware_concurrency();
        if (numThreads <= 0) numThreads = 1;
    }
    m_numThreads = numThreads;

    // 队列与计数数组原地构造（含 atomic 成员，vector 无法搬运，用 unique_ptr 数组）
    m_queues = std::make_unique<WorkStealingQueue[]>(m_numThreads);
    m_stealCounts = std::make_unique<std::atomic<uint64_t>[]>(m_numThreads); // 值初始化为 0
    m_workerBusyNs = std::make_unique<std::atomic<uint64_t>[]>(m_numThreads); // 负载分布诊断

    m_threads.reserve(m_numThreads);
    for (int i = 0; i < m_numThreads; ++i) {
        m_threads.emplace_back([this, i]() { WorkerLoop(i); });
    }
}

ThreadPool::~ThreadPool()
{
    Shutdown();
}

void ThreadPool::Shutdown()
{
    // 幂等：重复调用直接返回
    if (m_shutdown.load(std::memory_order_acquire)) {
        if (m_threads.empty()) return;
    }

    m_shutdown.store(true, std::memory_order_release);

    // 平稳 join：工作线程自会消费完所有待办任务后退出
    for (std::thread& t : m_threads) {
        if (t.joinable()) t.join();
    }
    m_threads.clear();
}

void ThreadPool::Submit(const std::function<void()>& task)
{
    const uint32_t w = m_nextWorker.fetch_add(1, std::memory_order_relaxed) % (uint32_t)m_numThreads;
    SubmitTo((int)w, task);
}

void ThreadPool::SubmitTo(int workerId, const std::function<void()>& task)
{
    WorkStealingQueue& q = m_queues[workerId];
    while (!q.Push(task)) {
        std::this_thread::yield(); // 环形队列满：退让后重试
    }
}

bool ThreadPool::TrySteal(int workerId, Task& task)
{
    // 随机挑选起始受害者，环形轮询一圈（防扎堆偷同一个队列）
    const int n = m_numThreads;
    int victim = (int)(nextRand() % (uint32_t)n);
    for (int i = 0; i < n; ++i) {
        if (victim != workerId && m_queues[victim].Steal(task)) {
            if (workerId >= 0) {
                m_stealCounts[workerId].fetch_add(1, std::memory_order_relaxed);
            }
            m_stealCount.fetch_add(1, std::memory_order_relaxed);
            return true;
        }
        victim = (victim + 1) % n;
    }
    return false;
}

bool ThreadPool::AllQueuesEmpty() const
{
    for (int i = 0; i < m_numThreads; ++i) {
        if (!m_queues[i].IsEmpty()) return false;
    }
    return true;
}

void ThreadPool::WorkerLoop(int workerId)
{
    // 用线程身份混合随机种子：各 worker 窃取起点序列互不相同
    t_rngState = 0x9E3779B9u ^ (uint32_t)(workerId + 1) * 0x85EBCA6Bu;
    WorkStealingQueue& local = m_queues[workerId];
    int spin = 0;

    while (true) {
        Task task;

        // 1. 优先消费本地队列（LIFO：缓存最热）
        if (local.Pop(task)) {
            if (task) { // 防御：绝不调用空任务
                const auto t0 = std::chrono::steady_clock::now();
                task();
                m_workerBusyNs[workerId].fetch_add(
                    (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t0).count(),
                    std::memory_order_relaxed);
                m_executedCount.fetch_add(1, std::memory_order_relaxed);
            }
            spin = 0;
            continue;
        }

        // 2. 本地为空：随机轮询窃取邻居（FIFO：老任务颗粒度大）
        if (TrySteal(workerId, task)) {
            if (task) { // 防御：绝不调用空任务
                const auto t0 = std::chrono::steady_clock::now();
                task();
                m_workerBusyNs[workerId].fetch_add(
                    (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t0).count(),
                    std::memory_order_relaxed);
                m_executedCount.fetch_add(1, std::memory_order_relaxed);
            }
            spin = 0;
            continue;
        }

        // 3. 全局无活：检查退出条件
        if (m_shutdown.load(std::memory_order_relaxed) && AllQueuesEmpty()) {
            break;
        }

        // 4. 轻量自旋退让：先 _mm_pause 自旋（任务间隔短，免上下文切换），
        //    超过阈值再让出时间片，防无任务时烧满 CPU
        if (++spin < 200) {
            _mm_pause();
        } else {
            std::this_thread::yield();
            spin = 0;
        }
    }
}

void ThreadPool::Wait(std::atomic<int>& counter)
{
    // 主线程化身 Worker：计数器归零前不停窃取干活——0 微秒休眠调度浪费
    t_rngState = 0x9E3779B9u ^ 0x85EBCA6Bu;
    int spin = 0;
    while (counter.load(std::memory_order_acquire) > 0) {
        Task task;
        if (TrySteal(-1, task)) {
            if (task) { // 防御：绝不调用空任务
                const auto t0 = std::chrono::steady_clock::now();
                task();
                m_mainBusyNs.fetch_add(
                    (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - t0).count(),
                    std::memory_order_relaxed);
                m_mainExecuted.fetch_add(1, std::memory_order_relaxed);
            }
            spin = 0;
            continue;
        }
        if (++spin < 200) {
            _mm_pause();
        } else {
            std::this_thread::yield();
            spin = 0;
        }
    }
}

void ThreadPool::ParallelFor(int start, int end, const std::function<void(int)>& func)
{
    const int total = end - start;
    if (total <= 0) return;

    // 区间切分为子批次：每核 4 批（总批数封顶任务总数），推入各核队列并发执行
    const int batchCount = total < m_numThreads * 4 ? total : m_numThreads * 4;
    std::atomic<int> counter(batchCount);

    const int base = total / batchCount;
    const int rem = total % batchCount;
    int lo = start;
    for (int b = 0; b < batchCount; ++b) {
        const int hi = lo + base + (b < rem ? 1 : 0);
        const int batchLo = lo, batchHi = hi;
        SubmitTo(b % m_numThreads, [&counter, &func, batchLo, batchHi]() {
            for (int i = batchLo; i < batchHi; ++i) {
                func(i);
            }
            counter.fetch_sub(1, std::memory_order_release);
        });
        lo = hi;
    }

    // 协作等待：主线程也参与窃取执行，全部批次完成后返回
    Wait(counter);
}

} // namespace Physics2D
