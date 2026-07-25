#include "BroadPhase.h"
#include <algorithm>
void BroadPhase::UpdatePairs(BroadPhaseCallback callback)
{
	m_pairBuffer.clear();
	for (int32_t proxyIdA : m_moveBuffer)
	{
		if (proxyIdA == -1 || !m_tree.IsLeaf(proxyIdA)) {
			continue;
		}
		const AABB& fatAABB = m_tree.GetNodeAABB(proxyIdA);
		auto treeCallback = [&](int32_t proxyIdB) -> bool {
			if (proxyIdA == proxyIdB)return true;//排除自己
			Pair pair;
			pair.proxyIdA = std::min(proxyIdA, proxyIdB);
			pair.proxyIdB = std::max(proxyIdA, proxyIdB);

			m_pairBuffer.push_back(pair);
			return true;
			};
		m_tree.Query(fatAABB, treeCallback);
	}
	// 2. 排序并去重
	sort(m_pairBuffer.begin(), m_pairBuffer.end());
	m_pairBuffer.erase(std::unique(m_pairBuffer.begin(), m_pairBuffer.end()), m_pairBuffer.end());
	// 3. 触发外部回调
	for (const auto& pair : m_pairBuffer) {
		void* userDataA = m_tree.GetUserData(pair.proxyIdA);
		void* userDataB = m_tree.GetUserData(pair.proxyIdB);
		callback(userDataA, userDataB);
	}

	// 4. 清空移动缓冲区，准备下一帧
	m_moveBuffer.clear();
}

void BroadPhase::DestroyProxy(int32_t proxyId) {
	m_tree.DestroyProxy(proxyId);
	// 这里不做多余操作，直接删树里的
}

int32_t BroadPhase::CreateProxy(const AABB& aabb, void* userData) {
	// 1. 调用底层的树去创建
	int32_t proxyId = m_tree.CreateProxy(aabb, userData);

	// 2. 关键：把新创建的物体标记为“移动过”
	// 这样在接下来的 UpdatePairs 中，它会立即和树里的其他物体查一遍碰撞
	BufferMove(proxyId);

	return proxyId;
}
bool BroadPhase::MoveProxy(int32_t proxyId, const AABB& aabb, const Vector2& displacement) {
	// 1. 调用底层的树去尝试移动
	bool changed = m_tree.MoveProxy(proxyId, aabb, displacement);

	// 2. 如果树告诉我们：它跳出了肥包围盒，结构变了
	if (changed) {
		// 那就把它放进待查询名单
		BufferMove(proxyId);
	}
	return changed;
}

void BroadPhase::BufferMove(int32_t proxyId) {
	// 把移动过的物体索引放进 vector
	m_moveBuffer.push_back(proxyId);
}