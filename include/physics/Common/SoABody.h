#pragma once
#include "Vector2.h"
#include <cstdint>
#include <vector>

class Body;

/// @brief 刚体热数据 SoA 容器（V5 Day 2：数据导向设计改造）
///
/// 把每帧高频迭代的动力学数据从散落在堆各处的 Body 对象中剥离出来，
/// 按属性分门别类排成连续、平铺的平行数组（Structure of Arrays）：
///   - 一条 64B 缓存行一次可供给 4 个刚体的 (x,y) 位置连续推进；
///   - 积分循环告别指针追逐（Pointer Chasing），L1/L2 命中率拉满；
///   - 为明日 AVX2 SIMD 矢量化铺路（std::vector 首地址在 x64 上 16 字节对齐）。
///
/// 槽位管理采用 Swap-and-Pop：销毁 index i 的刚体时把末尾槽位整体搬运
/// 覆盖到 i，再 pop_back 截断——严格 O(1)，数组始终 100% 紧致无空洞。
class BodySoA
{
public:
    // 紧凑位标记（每刚体 1 个 uint32_t，替代散落的 bool）
    static const uint32_t FLAG_AWAKE  = 1u << 0; // 清醒（参与积分与求解）
    static const uint32_t FLAG_BULLET = 1u << 1; // 高速刚体（CCD 扫掠）

    /// 为新刚体分配槽位：追加所有平行数组，写入热数据快照，
    /// 并把槽位句柄反向写回 Body::m_soaIndex。返回槽位索引。
    int AllocateSlot(Body* body);
    /// Swap-and-Pop 极速移除：末尾槽位整体搬运覆盖 index，被挪动刚体的
    /// 句柄自动同步；被移除刚体彻底脱离 SoA（句柄重置 -1，杜绝悬空访问）。
    void FreeSlot(int index);

    inline int GetCount() const { return (int)bodyOwners.size(); }
    inline Body* GetOwner(int index) const { return bodyOwners[index]; }

    // --- 平行紧致数组（热数据：每帧积分/求解高频迭代） ---
    std::vector<Vector2> positions;    // 世界坐标 (x, y)
    std::vector<Vector2> velocities;   // 线速度 (vx, vy)
    std::vector<float>   rotations;    // 旋转角（弧度）
    std::vector<float>   angularVels;  // 角速度
    std::vector<float>   invMasses;    // 质量倒数（静态刚体恒为 0）
    std::vector<float>   invInertias;  // 转动惯量倒数
    std::vector<Vector2> forces;       // 累积外力（每帧积分后清零）
    std::vector<float>   torques;      // 累积扭矩
    std::vector<float>   gravityScales;// 每体重力缩放（积分公式需要，规格清单外的必要补充）
    std::vector<uint32_t> flags;       // 紧凑位标记（FLAG_AWAKE / FLAG_BULLET）
    std::vector<Body*>   bodyOwners;   // 槽位 -> 刚体句柄反向映射表

private:
    // 把 from 槽位的全部热数据整体搬运到 to 槽位，并同步句柄映射
    void MoveSlot(int from, int to);
};
