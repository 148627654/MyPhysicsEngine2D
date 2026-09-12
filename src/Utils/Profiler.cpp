#include "Profiler.h"
#include <sstream>
#include <iomanip> 
void Profiler::Start(const std::string& name)
{
	m_startTimes[name] = std::chrono::high_resolution_clock::now();
}

void Profiler::Stop(const std::string& name)
{
	// 1. 获取结束时刻
	auto endTime = std::chrono::high_resolution_clock::now();
	// 2. 查找开始时刻
	auto itStart = m_startTimes.find(name);
	if (itStart == m_startTimes.end()) {
		return; // 容错：如果没调 Start 直接调 Stop，忽略
	}
	// 3. 计算差值 (单位：毫秒 ms)
	std::chrono::duration<float, std::milli> duration = endTime - itStart->second;
	float elapsedMs = duration.count();
	// 4. 更新记录
	ProfileRecord& record = m_records[name];
	record.accumulatedTime += elapsedMs; // 累加本帧内的耗时
	record.callCount++;                  // 记录本帧调用次数

	// 5. 移除本次开始标记，防止误用
	m_startTimes.erase(itStart);
}

void Profiler::BeginFrame()
{
	for (auto& pair : m_records) {
		pair.second.accumulatedTime = 0.0f;
		pair.second.callCount = 0;
	}
}

void Profiler::EndFrame() {
	for (auto& pair : m_records) {
		ProfileRecord& rec = pair.second;

		// 这一帧的总耗时就是累积值
		rec.lastTime = rec.accumulatedTime;

		// 更新历史峰值
		if (rec.lastTime > rec.maxTime) {
			rec.maxTime = rec.lastTime;
		}

		// 计算移动平均值 (Moving Average)
		rec.averageTime = rec.averageTime * m_smoothing + rec.lastTime * (1.0f - m_smoothing);
	}
}

void Profiler::SetCounter(const std::string& name, int value) {
	m_counters[name] = value;
}

// 打印完整的性能分析报告
void Profiler::PrintReport() {
	std::stringstream ss;

	// --- 报表表头 ---
	ss << "\n" << std::string(60, '=') << "\n";
	ss << "          PHYSICS ENGINE PERFORMANCE REPORT (V2)          \n";
	ss << std::string(60, '-') << "\n";

	// --- 1. 耗时统计 (Timings) ---
	// 设置列宽和对齐方式
	ss << std::left << std::setw(20) << "Task Name"
		<< std::setw(12) << "Avg (ms)"
		<< std::setw(12) << "Max (ms)"
		<< "Call/Frame\n";
	ss << std::string(60, '.') << "\n";

	for (auto const& item : m_records) {
		const std::string& name = item.first;
		const ProfileRecord& rec = item.second;

		ss << std::left << std::setw(20) << name
			<< std::fixed << std::setprecision(3) // 保留三位小数
			<< std::setw(12) << rec.averageTime
			<< std::setw(12) << rec.maxTime
			<< rec.callCount << "\n";
	}

	// --- 2. 状态统计 (Counters) ---
	if (!m_counters.empty()) {
		ss << std::string(60, '-') << "\n";
		ss << "ENGINE STATS:\n";
		for (auto const& item : m_counters) {
			ss << "  > " << std::left << std::setw(18) << item.first
				<< ": " << item.second << "\n";
		}
	}

	ss << std::string(60, '=');

	// --- 3. 最终通过 Logger 输出 ---
	Logger::Info(ss.str());
}
