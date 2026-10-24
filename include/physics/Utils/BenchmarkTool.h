#pragma once
// BenchmarkTool.h — 微秒级性能基准评估套件（V5 Day 4）
//
// 工程原则："建立基准 → 定位瓶颈 → 实施优化 → 数据回归"闭环，绝不靠
// "感觉变快了"。套件提供：
//   - BenchmarkTimer：high_resolution_clock 纳秒级掐表；
//   - BenchmarkSuite：命名用例注册 + 预热运行（CPU 睿频激活/缓存稳定态）
//     + 多轮采样 + 统计学分析（均值/中位数/最小/最大/标准差）；
//   - CSV 数据管道：一键导出性能报表，Excel/Python 直接绘图。
#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace Physics2D {

struct BenchmarkResult {
    std::string name;        // 用例名
    int objectCount = 0;     // 场景规模（刚体数量 / 操作次数档位）
    double meanMs = 0.0;     // 平均耗时
    double medianMs = 0.0;   // 中位数（抗调度噪声，主判据）
    double minMs = 0.0;      // 最小耗时
    double maxMs = 0.0;      // 最大耗时
    double stddevMs = 0.0;   // 标准差
};

/// 纳秒级计时器：Start() 掐表，Stop 返回毫秒/微秒
class BenchmarkTimer {
public:
    void Start() { m_start = std::chrono::high_resolution_clock::now(); }
    double StopMs();
    double StopUs();
private:
    std::chrono::high_resolution_clock::time_point m_start;
};

/// 基准套件：预热 + 多轮采样 + 统计学分析 + CSV 导出
class BenchmarkSuite {
public:
    explicit BenchmarkSuite(const std::string& suiteName) : m_name(suiteName) {}

    /// 运行一个基准用例：先 warmupRuns 轮预热（不计时），再 measureRuns 轮
    /// 采样计时；返回中位数/均值/极值/标准差齐全的结果（同时存入内部结果表）
    BenchmarkResult RunBenchmark(const std::string& caseName, int objectCount,
        int warmupRuns, int measureRuns, const std::function<void()>& task);

    /// 将全部结果序列化导出为 CSV（含表头；自动创建输出目录）
    void ExportToCSV(const std::string& filePath) const;

    inline const std::vector<BenchmarkResult>& GetResults() const { return m_results; }

private:
    std::string m_name;
    std::vector<BenchmarkResult> m_results;
};

} // namespace Physics2D
