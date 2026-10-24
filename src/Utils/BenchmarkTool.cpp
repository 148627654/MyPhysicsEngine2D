#include "BenchmarkTool.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

namespace Physics2D {

double BenchmarkTimer::StopMs()
{
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - m_start).count();
}

double BenchmarkTimer::StopUs()
{
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(end - m_start).count();
}

BenchmarkResult BenchmarkSuite::RunBenchmark(const std::string& caseName, int objectCount,
    int warmupRuns, int measureRuns, const std::function<void()>& task)
{
    BenchmarkResult result;
    result.name = caseName;
    result.objectCount = objectCount;

    // 1. 预热运行：CPU 睿频激活、分支预测器/页表/L1~L3 缓存进入稳定态
    for (int i = 0; i < warmupRuns; ++i) {
        task();
    }

    // 2. 多轮采样：逐轮掐表
    std::vector<double> samples;
    samples.reserve(measureRuns);
    BenchmarkTimer timer;
    for (int i = 0; i < measureRuns; ++i) {
        timer.Start();
        task();
        samples.push_back(timer.StopMs());
    }

    // 3. 统计学分析：均值 / 中位数（排序拷贝）/ 极值 / 标准差
    double sum = 0.0;
    for (double s : samples) sum += s;
    result.meanMs = sum / (double)samples.size();
    result.minMs = *std::min_element(samples.begin(), samples.end());
    result.maxMs = *std::max_element(samples.begin(), samples.end());

    std::vector<double> sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    const size_t n = sorted.size();
    result.medianMs = (n % 2 == 1) ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5;

    double sqSum = 0.0;
    for (double s : samples) sqSum += (s - result.meanMs) * (s - result.meanMs);
    result.stddevMs = std::sqrt(sqSum / (double)samples.size());

    m_results.push_back(result);
    return result;
}

void BenchmarkSuite::ExportToCSV(const std::string& filePath) const
{
    // 自动创建输出目录（多级）
    const std::filesystem::path path(filePath);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream file(filePath, std::ios::out | std::ios::trunc);
    if (!file.is_open()) return;

    file << "Suite,Case,ObjectCount,MeanMs,MedianMs,MinMs,MaxMs,StdDevMs\n";
    for (const BenchmarkResult& r : m_results) {
        file << m_name << "," << r.name << "," << r.objectCount << ","
             << r.meanMs << "," << r.medianMs << "," << r.minMs << ","
             << r.maxMs << "," << r.stddevMs << "\n";
    }
    file.close();
}

} // namespace Physics2D
