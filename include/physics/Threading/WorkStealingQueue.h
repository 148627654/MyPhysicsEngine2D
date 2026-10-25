#pragma once
// WorkStealingQueue.h — 低延迟工作窃取队列（V5 Day 5）
//
// Vyukov 有界 MPMC 环形队列（1024cores 设计，每槽序列号仲裁）：
//   - Push/Pop 分别对 pushPos/popPos 做 fetch_add/CAS 占位，多生产者
//     多消费者安全——本线程池中"主线程推入、工作线程弹出、他线程窃取"
//     三者并发写队列，经典 Chase-Lev（bottom 属主单线程假设）会丢失更新；
//   - 每槽位序列号：写入完成前消费者读到 stale 序列号自动让位/判空，
//     数据竞争由 acquire/release 配对杜绝；
//   - alignas(64) 缓存行对齐：消除多核伪共享总线风暴。
#include <atomic>
#include <cstdint>
#include <functional>

namespace Physics2D {

class alignas(64) WorkStealingQueue {
public:
    static const int CAPACITY = 1024; // 固定槽位数（环形，2 的幂）

    using Task = std::function<void()>;

    WorkStealingQueue() : m_pushPos(0), m_popPos(0)
    {
        // 序列号初始化：槽位 i 期待第 i 次推入（写完成后翻到 i+1）
        for (int i = 0; i < CAPACITY; ++i) {
            m_sequence[i].store((uint32_t)i, std::memory_order_relaxed);
        }
    }

    /// 推入任务（队列满返回 false，调用方重试）；任意线程可并发调用
    bool Push(const Task& task)
    {
        uint32_t pos = m_pushPos.load(std::memory_order_relaxed);
        for (;;) {
            const uint32_t cell = pos % CAPACITY;
            const uint32_t seq = m_sequence[cell].load(std::memory_order_acquire);
            const intptr_t diff = (intptr_t)seq - (intptr_t)pos;
            if (diff == 0) {
                if (m_pushPos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    m_buffer[cell] = task; // 写数据
                    m_sequence[cell].store(pos + 1, std::memory_order_release); // 发布
                    return true;
                }
            } else if (diff < 0) {
                return false; // 队列满
            } else {
                pos = m_pushPos.load(std::memory_order_relaxed); // 被抢位，重读
            }
        }
    }

    /// 弹出任务（空返回 false）；任意线程可并发调用
    bool Pop(Task& task)
    {
        uint32_t pos = m_popPos.load(std::memory_order_relaxed);
        for (;;) {
            const uint32_t cell = pos % CAPACITY;
            const uint32_t seq = m_sequence[cell].load(std::memory_order_acquire);
            const intptr_t diff = (intptr_t)seq - (intptr_t)(pos + 1);
            if (diff == 0) {
                if (m_popPos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
                    task = m_buffer[cell];
                    // 重置序列号：环回后第 pos+CAPACITY 次推入可复用此槽
                    m_sequence[cell].store(pos + CAPACITY, std::memory_order_release);
                    return true;
                }
            } else if (diff < 0) {
                return false; // 队列空（或写入未完成）
            } else {
                pos = m_popPos.load(std::memory_order_relaxed); // 被抢位，重读
            }
        }
    }

    /// 窃取任务：MPMC 队列下与 Pop 同构（全 FIFO），接口独立保留语义
    bool Steal(Task& task) { return Pop(task); }

    /// 近似判空（终止检测用；写入未完成的瞬间可能误判空，Shutdown 前
    /// 所有任务均已 Wait 归零，无影响）
    bool IsEmpty() const
    {
        return m_pushPos.load(std::memory_order_relaxed) <= m_popPos.load(std::memory_order_relaxed);
    }

private:
    std::atomic<uint32_t> m_pushPos; // 推入游标（生产者 fetch_add 占位）
    std::atomic<uint32_t> m_popPos;  // 弹出游标（消费者 CAS 占位）
    std::atomic<uint32_t> m_sequence[CAPACITY]; // 每槽序列号仲裁
    Task m_buffer[CAPACITY]; // 固定环形槽位
};

} // namespace Physics2D
