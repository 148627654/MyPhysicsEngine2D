#pragma once

#include "../Events/ContactKey.h"
#include "../collision/Manifold.h"
#include <unordered_map>
#include <vector>

class ContactManager
{
public:
    ContactManager() = default;
    ~ContactManager() = default;

    // 当任意形状发生碰撞时，由窄相检测器调用
    void AddContact(Body* bodyA, Body* bodyB, const Manifold& manifold, bool isTrigger);
    // 在窄相检测结束、冲量求解之前调用，对比上一帧与当前帧集合
    void UpdateStates();
    // 在 World::Step 的最末尾调用
    void EndFrame();
    // 当刚体从世界中移除时调用，清除所有涉及该 Body 的历史缓存，杜绝野指针
    void OnBodyDestroyed(Body* body);
    // 获取本帧推导出的所有生命周期记录 (包含 Enter, Stay, Exit)
    const std::vector<ContactRecord>& GetLifecycleRecords() const {
        return m_lifecycleRecords;
    }

    // 重置清空所有缓存 (在 World::Clear 时使用)
    void Clear();
private:
    // 上一帧与当前帧的双缓冲映射表
    std::unordered_map<ContactKey, ContactRecord, ContactKeyHash> m_previousContacts;
    std::unordered_map<ContactKey, ContactRecord, ContactKeyHash> m_currentContacts;

    // 本帧生成的完整事件/记录扁平列表
    std::vector<ContactRecord> m_lifecycleRecords;
};