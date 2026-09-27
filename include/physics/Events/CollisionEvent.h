#pragma once

#include "../Dynamics/Body.h"
#include <vector>
#include "../Common/Vector2.h"

// 实体碰撞事件数据包
struct CollisionEvent {
    Body* bodyA = nullptr;
    Body* bodyB = nullptr;
    Vector2 normal;                  // 碰撞法线 (A -> B)
    std::vector<Vector2> contacts;   // 接触点列表 (至多 2 个)
    float maxImpulse = 0.0f;         // 碰撞法向最大冲量 (用于判定冲击力)
};

// 触发器事件数据包
struct TriggerEvent {
    Body* triggerBody = nullptr;     // 触发器刚体
    Body* otherBody = nullptr;       // 穿过触发器的刚体
};