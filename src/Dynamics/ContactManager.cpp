#include "ContactManager.h"
#include <utility>
void ContactManager::addContact(Body* bodyA, Body* bodyB, const Manifold& manifold, bool isTrigger)
{
    // 1.任何一个为空，或者自己撞自己，立刻退出
    if (!bodyA || !bodyB || bodyA == bodyB) {
        return;
    }

    // 2. 无有效触点则不记录 (或者只在有接触时记录)
    if (manifold.contactCount <= 0) {
        return;
    }

    // 3. 构建无序唯一键
    ContactKey key(bodyA->getProxyId(), bodyB->getProxyId());

    // 4. 打包当前帧记录并存入哈希表
    ContactRecord record;
    record.bodyA = bodyA;
    record.bodyB = bodyB;
    record.isTrigger = isTrigger;
    record.manifold = manifold;
    record.state = ContactState::Enter; // 临时默认值，后续由 updateStates 最终裁决

    m_currentContacts[key] = record;
}

void ContactManager::updateStates()
{
    m_lifecycleRecords.clear();

    // 1. 正向扫描：用 C++11 范围 for 循环代替繁琐的迭代器
    for (auto& pair : m_currentContacts)
    {
        const auto& key = pair.first;
        auto& currentRecord = pair.second;

        // 用三元表达式直接定性，合并重复的 push_back
        bool wasTouching = (m_previousContacts.find(key) != m_previousContacts.end());
        currentRecord.state = wasTouching ? ContactState::Stay : ContactState::Enter;

        m_lifecycleRecords.push_back(currentRecord);
    }

    // 2. 反向扫描：找 Exit
    for (auto& pair : m_previousContacts)
    {
        const auto& key = pair.first;
        auto& prevRecord = pair.second;

        // 如果当前帧没了，说明刚分开
        if (m_currentContacts.find(key) == m_currentContacts.end())
        {
            prevRecord.state = ContactState::Exit;
            m_lifecycleRecords.push_back(prevRecord);
        }
    }
}

void ContactManager::endFrame()
{
    // 帧末尾，把当前map“移交”给上一帧
    m_previousContacts = std::move(m_currentContacts);

	m_currentContacts.clear();
}

void ContactManager::onBodyDestroyed(Body* body)
{
    if (!body)
        return;

    // 1. 遍历当前帧，用迭代器安全删除
    for (auto it = m_currentContacts.begin(); it != m_currentContacts.end(); )
    {
        if (it->second.bodyA == body || it->second.bodyB == body)
            it = m_currentContacts.erase(it); // 接住下一个有效迭代器
        else
            ++it;
    }

    // 2. 遍历上一帧，用迭代器安全删除
    for (auto it = m_previousContacts.begin(); it != m_previousContacts.end(); )
    {
        if (it->second.bodyA == body || it->second.bodyB == body)
            it = m_previousContacts.erase(it);
        else
            ++it;
    }

    // 3. 遍历生命周期记录（你原本写的满分代码，保持不变）
    for (auto it = m_lifecycleRecords.begin(); it != m_lifecycleRecords.end(); )
    {
        if (it->bodyA == body || it->bodyB == body)
            it = m_lifecycleRecords.erase(it);
        else
            ++it;
    }
}

void ContactManager::clear()
{
    m_previousContacts.clear();
    m_currentContacts.clear();
    m_lifecycleRecords.clear();
}