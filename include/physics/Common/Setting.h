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
	constexpr float BIAS = 0.6f;
	//bias 建议设为 `0.2f` 到 `0.8f`（代表每帧修复百分之几）。
	

}