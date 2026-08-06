#pragma once
#include "Vector2.h"
namespace Settings
{
    // 固定时间步 60FPS
    static constexpr float DT = 1.0f / 60.0f;
    // 标准重力
    static constexpr Vector2 GRAVITY(0, -9.8f);
    // Π
    static constexpr float PAI = 3.1415926f;

    //slop 建议设为 `0.01f` 到 `0.05f`
    static constexpr float PENETRATION_ALLOWANCE = 0.02f;
    constexpr float BIAS = 0.2f;
    //bias 建议设为 `0.2f` 到 `0.8f`（代表每帧修复百分之几）。


    static constexpr float k_aabbExtension = 0.1f;      //单位长度
    static constexpr float k_aabbMultiplier = 2.0f;     //位移预测倍率

    static constexpr float LinearSleepThreshold = 0.2f;
    static constexpr float AngularSleepThreshold = 0.2f;  //(角速度阈值)
    static constexpr float TimeToSleep = 0.5f;
    static constexpr float EPSILON = 1e-7f;

    // 2. 线性容差 (Linear Slop)
    static constexpr float LINEAR_SLOP = 0.005f;

    // 3. 角度容差 (Angular Slop)
    static constexpr float ANGULAR_SLOP = (2.0f / 180.0f * 3.1415926f);

    // 4. TOI 专用容差
    static constexpr float TOI_BAUMGARTE = 0.75f; // TOI 修正强度
    static constexpr float TOI_SLOP = 8.0f * EPSILON;
};