#pragma once
#include <string>
#include <map>
#include <chrono>
#include <vector>
#include "Logger.h"
struct ProfileRecord {
    float lastTime = 0.0f;      // 上一次耗时 (ms)
    float accumulatedTime = 0.0f; // 本帧累计耗时 (用于处理多次调用)
    float averageTime = 0.0f;   // 历史平均耗时 (ms)
    float maxTime = 0.0f;       // 峰值耗时 (ms)
    int callCount = 0;          // 本帧被调用次数
};


class Profiler
{
public:
    Profiler()=default;
    // --- 核心打点接口 ---
    void Start(const std::string& name);
    void Stop(const std::string& name);
    // --- 帧生命周期管理 ---
    void BeginFrame(); // 每一帧 Step 开始前调用，重置累计值
    void EndFrame();   // 每一帧 Step 结束后调用，计算平均值
    // --- 数据统计与输出 ---
    void PrintReport();
    const std::map<std::string, ProfileRecord>& GetRecords() const { return m_records; }

    // 计数器接口（记录物体数量、碰撞对数量等）
    void SetCounter(const std::string& name, int value);

private:
    std::map<std::string, ProfileRecord> m_records;
    std::map<std::string, std::chrono::high_resolution_clock::time_point> m_startTimes;
    std::map<std::string, int> m_counters;

    const float m_smoothing = 0.95f; // 用于计算移动平均值的平滑系数
};

// 只要定义这个变量，就会自动开始计时，离开作用域自动停止
class ScopedTimer {
public:
    ScopedTimer(Profiler& profiler, const std::string& name)
        : m_profiler(profiler), m_name(name) {
        m_profiler.Start(m_name);
    }
    ~ScopedTimer() {
        m_profiler.Stop(m_name);
    }
private:
    Profiler& m_profiler;
    std::string m_name;
};