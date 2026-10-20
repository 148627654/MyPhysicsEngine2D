#pragma once
// SIMDMath.h — AVX2/SSE 指令集封装与平台检测（V5 Day 3）
//
// 工程规范：Debug|x64 两工程均开启 /arch:AVX2（vcxproj 的
// EnableEnhancedInstructionSet = AdvancedVectorExtensions2），
// 编译器定义 __AVX2__。本头提供：
//   - SIMD_AVX2_ENABLED 编译期开关（未开 /arch:AVX2 时自动标量回退）
//   - IsAVX2Supported() 运行时 CPUID 检测（跨机器安全降级）
#include <immintrin.h>
#include <intrin.h>

#if defined(__AVX2__)
    #define SIMD_AVX2_ENABLED 1
#else
    #define SIMD_AVX2_ENABLED 0
#endif

namespace Physics2D {
namespace SIMD {

/// 运行时 CPU 能力检测（只执行一次，结果缓存）：
///   - CPUID.1:ECX 位 28 = AVX、位 27 = OSXSAVE
///   - XGETBV(0) 位 1:0 = 11b（OS 已启用 XMM/YMM 状态保存）
///   - CPUID.7:EBX 位 5 = AVX2
inline bool IsAVX2Supported()
{
    static const bool supported = []() {
        int cpuInfo[4] = { 0, 0, 0, 0 };
        __cpuidex(cpuInfo, 1, 0);
        const bool avxAndOsxsave = (cpuInfo[2] & (1 << 28)) != 0 && (cpuInfo[2] & (1 << 27)) != 0;
        if (!avxAndOsxsave)
            return false;
        const bool osSavesYmm = (_xgetbv(0) & 0x6) == 0x6;
        if (!osSavesYmm)
            return false;
        __cpuidex(cpuInfo, 7, 0);
        return (cpuInfo[1] & (1 << 5)) != 0; // EBX bit 5 = AVX2
    }();
    return supported;
}

} // namespace SIMD
} // namespace Physics2D
