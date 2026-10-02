#pragma once
#include "CollisionEvent.h"

/// @brief 接触事件监听器（推送式回调）
///
/// 实现本接口并注册到 World（setContactListener）即可收到事件：
///  - onCollisionEnter/Stay/Exit：实体碰撞（携带法线与接触点）
///  - onTriggerEnter/Stay/Exit：触发器传感（无物理响应）
///
/// 回调在 step 末尾触发；回调内可安全调用 world.destroyBody（延迟销毁队列）。
/// 所有回调都有空默认实现，只需覆写关心的部分。
class ContactListener {
public:
    virtual ~ContactListener() = default;

    virtual void onCollisionEnter(const CollisionEvent& event) {}
    virtual void onCollisionStay(const CollisionEvent& event) {}
    virtual void onCollisionExit(const CollisionEvent& event) {}
    virtual void onTriggerEnter(const TriggerEvent& event) {}
    virtual void onTriggerStay(const TriggerEvent& event) {}
    virtual void onTriggerExit(const TriggerEvent& event) {}
};