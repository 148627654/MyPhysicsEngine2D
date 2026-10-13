#pragma once
#include "../Dynamics/Body.h"

/// @brief 射线命中结果数据包（World::RayCast 单目标接口的输出）
struct RayCastHit {
    Body* body = nullptr;     // 被命中的刚体
    Shape* shape = nullptr;   // 被命中的形状
    Vector2 point;            // 撞击点的世界绝对坐标
    Vector2 normal;           // 撞击表面朝外的单位法线
    float fraction = 0.0f;    // 撞击点在射线上的比例 [0, 1]
};
