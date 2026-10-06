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
    constexpr float BIAS = 0.1f;
    //bias 建议设为 `0.2f` 到 `0.8f`（代表每帧修复百分之几）。

    // Restitution takes effect only above this relative normal velocity (m/s)
    static constexpr float RestitutionVelocityThreshold = 1.0f;


    static constexpr float k_aabbExtension = 0.1f;      //单位长度
    static constexpr float k_aabbMultiplier = 2.0f;     //位移预测倍率

    // 睡眠阈值（速度量纲 m/s、rad/s，与 Box2D 对齐）：
    // 【修复】原值 1.0 当"速度平方"比较 → 任何有实体接触且 |v| < 1 m/s 的物体
    // 每帧被清零速度，缓慢滑动/滑行被冻成"清零-重加速"棘轮（布娃娃缓降 bug 同源）。
    // Box2D 对应 b2_linearSleepTolerance = 0.01 m/s
    static constexpr float LinearSleepThreshold = 0.01f;
    static constexpr float AngularSleepThreshold = 0.8f;  //(角速度阈值)
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