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
}