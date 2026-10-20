#pragma once
#include "Vector2.h"
#include <cstdint>
#include <cstdlib>
#include <new>
#include <vector>
#include <malloc.h> // _aligned_malloc / _aligned_free（MSVC）

class Body;

/// 32 字节对齐分配器（AVX2 铁律）：_mm256_load_ps/_mm256_store_ps 要求
/// 目标地址 32 字节物理对齐，std::vector 默认分配器只保证 16——本分配器
/// 让 SoA 所有平行数组的首地址与每次扩容都满足 32 字节对齐
template <typename T>
struct AlignedAllocator32
{
    using value_type = T;
    AlignedAllocator32() = default;
    template <typename U> AlignedAllocator32(const AlignedAllocator32<U>&) {}
    T* allocate(std::size_t n)
    {
        void* p = _aligned_malloc(n * sizeof(T), 32);
        if (p == nullptr) throw std::bad_alloc();
        return static_cast<T*>(p);
    }
    void deallocate(T* p, std::size_t) { _aligned_free(p); }
    template <typename U> struct rebind { using other = AlignedAllocator32<U>; };
};
template <typename T, typename U>
inline bool operator==(const AlignedAllocator32<T>&, const AlignedAllocator32<U>&) { return true; }
template <typename T, typename U>
inline bool operator!=(const AlignedAllocator32<T>&, const AlignedAllocator32<U>&) { return false; }

/// @brief 刚体热数据 SoA 容器（V5 Day 2 建立，Day 3 SIMD 化改造）
///
/// Day 2：把高频动力学数据从散落堆各处的 Body 对象剥离为连续平铺平行数组，
/// 槽位管理采用 Swap-and-Pop（O(1) 无空洞），Body 访问器双向无感穿透。
/// Day 3：数组升级为"纯单精度拆分流"（positionsX/Y、velocitiesX/Y 等
/// 独立 float 数组）——AVX2 直接整条缓存行吞吐，免洗牌免转置；
/// 物理容量恒为 8 的倍数（SIMD 8 路批处理安全余量），首地址 32 字节对齐。
class BodySoA
{
public:
    // 紧凑位标记（每刚体 1 个 uint32_t，替代散落的 bool）
    static const uint32_t FLAG_AWAKE  = 1u << 0; // 清醒（参与积分与求解）
    static const uint32_t FLAG_BULLET = 1u << 1; // 高速刚体（CCD 扫掠）

    // ================= 槽位生命周期 =================
    int AllocateSlot(Body* body);
    void FreeSlot(int index);

    inline int GetCount() const { return (int)bodyOwners.size(); }
    inline Body* GetOwner(int index) const { return bodyOwners[index]; }

    // ================= 槽位数据访问（Body 穿透 / 诊断用） =================
    inline Vector2 GetPosition(int i) const { return Vector2(positionsX[i], positionsY[i]); }
    inline void SetPosition(int i, const Vector2& v) { positionsX[i] = v.x; positionsY[i] = v.y; }
    inline Vector2 GetVelocity(int i) const { return Vector2(velocitiesX[i], velocitiesY[i]); }
    inline void SetVelocity(int i, const Vector2& v) { velocitiesX[i] = v.x; velocitiesY[i] = v.y; }
    inline void AddVelocity(int i, const Vector2& dv) { velocitiesX[i] += dv.x; velocitiesY[i] += dv.y; }
    inline Vector2 GetForce(int i) const { return Vector2(forcesX[i], forcesY[i]); }
    inline void SetForce(int i, const Vector2& f) { forcesX[i] = f.x; forcesY[i] = f.y; }
    inline void AddForce(int i, const Vector2& f) { forcesX[i] += f.x; forcesY[i] += f.y; }
    inline void ClearForce(int i) { forcesX[i] = 0.0f; forcesY[i] = 0.0f; }
    inline float GetRotation(int i) const { return rotations[i]; }
    inline void SetRotation(int i, float r) { rotations[i] = r; }
    inline float GetAngularVel(int i) const { return angularVels[i]; }
    inline void SetAngularVel(int i, float w) { angularVels[i] = w; }
    inline void AddAngularVel(int i, float dw) { angularVels[i] += dw; }
    inline float GetInvMass(int i) const { return invMasses[i]; }
    inline void SetInvMass(int i, float m) { invMasses[i] = m; }
    inline float GetInvInertia(int i) const { return invInertias[i]; }
    inline void SetInvInertia(int i, float inv) { invInertias[i] = inv; }
    inline float GetTorque(int i) const { return torques[i]; }
    inline void SetTorque(int i, float t) { torques[i] = t; }
    inline void AddTorque(int i, float t) { torques[i] += t; }
    inline float GetGravityScale(int i) const { return gravityScales[i]; }
    inline void SetGravityScale(int i, float g) { gravityScales[i] = g; }
    inline void SetFlag(int i, uint32_t flag, bool on) { if (on) flags[i] |= flag; else flags[i] &= ~flag; }

    // ================= SIMD 连续单精度指针接口（AVX2 寄存器连续吞吐） =================
    inline float* GetPositionsX() { return positionsX.data(); }
    inline float* GetPositionsY() { return positionsY.data(); }
    inline float* GetVelocitiesX() { return velocitiesX.data(); }
    inline float* GetVelocitiesY() { return velocitiesY.data(); }
    inline float* GetRotations() { return rotations.data(); }
    inline float* GetAngularVels() { return angularVels.data(); }
    inline float* GetInvMasses() { return invMasses.data(); }
    inline float* GetInvInertias() { return invInertias.data(); }
    inline float* GetForcesX() { return forcesX.data(); }
    inline float* GetForcesY() { return forcesY.data(); }
    inline float* GetTorques() { return torques.data(); }
    inline float* GetGravityScales() { return gravityScales.data(); }

    // ================= 积分内核 =================
    /// AVX2 8 路并发前向欧拉积分（含标量尾部 + 分支掩码）。
    /// 运行时 CPU 不支持 AVX2 或 SetUseSIMD(false) 时自动降级 IntegrateScalar。
    void IntegrateSIMD(float dt, const Vector2& gravity);
    /// 标量回退路径（与旧版积分逐位一致，供对照/降级/验收）
    void IntegrateScalar(float dt, const Vector2& gravity);

    inline void SetUseSIMD(bool b) { m_useSIMD = b; }
    inline bool IsUsingSIMD() const { return m_useSIMD; }

    // --- 平行紧致数组（拆分单精度流：32 字节对齐 + 容量恒为 8 的倍数） ---
    std::vector<float, AlignedAllocator32<float>> positionsX;  // 世界坐标 X
    std::vector<float, AlignedAllocator32<float>> positionsY;  // 世界坐标 Y
    std::vector<float, AlignedAllocator32<float>> velocitiesX; // 线速度 X
    std::vector<float, AlignedAllocator32<float>> velocitiesY; // 线速度 Y
    std::vector<float, AlignedAllocator32<float>> rotations;   // 旋转角（弧度）
    std::vector<float, AlignedAllocator32<float>> angularVels; // 角速度
    std::vector<float, AlignedAllocator32<float>> invMasses;   // 质量倒数（静态恒 0）
    std::vector<float, AlignedAllocator32<float>> invInertias; // 转动惯量倒数
    std::vector<float, AlignedAllocator32<float>> forcesX;     // 累积外力 X（积分后清零）
    std::vector<float, AlignedAllocator32<float>> forcesY;     // 累积外力 Y
    std::vector<float, AlignedAllocator32<float>> torques;     // 累积扭矩
    std::vector<float, AlignedAllocator32<float>> gravityScales; // 每体重力缩放
    std::vector<uint32_t, AlignedAllocator32<uint32_t>> flags; // 位标记（AWAKE/BULLET）
    std::vector<Body*, AlignedAllocator32<Body*>> bodyOwners;  // 槽位 -> 刚体句柄

private:
    bool m_useSIMD = true; // 是否启用 AVX2 内核（测试可强制走标量对照路径）

    void MoveSlot(int from, int to);

    static inline int RoundUp8(int n) { return (n + 7) & ~7; }
    template <typename V>
    static void EnsureCapacity(V& v) { if (v.size() == v.capacity()) v.reserve(RoundUp8((int)v.size() + 1)); }
};
