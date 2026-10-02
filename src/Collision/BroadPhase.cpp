#include "BroadPhase.h"
#include <algorithm>
void BroadPhase::updatePairs(BroadPhaseCallback callback)
{
	m_pairBuffer.clear();
	for (int32_t proxyIdA : m_moveBuffer)
	{
		if (proxyIdA == -1 || !m_tree.isLeaf(proxyIdA)) {
			continue;
		}
		const AABB& fatAABB = m_tree.getNodeAABB(proxyIdA);
		auto treeCallback = [&](int32_t proxyIdB) -> bool {
			if (proxyIdA == proxyIdB)return true;//排除自己
			Pair pair;
			pair.proxyIdA = std::min(proxyIdA, proxyIdB);
			pair.proxyIdB = std::max(proxyIdA, proxyIdB);

			m_pairBuffer.push_back(pair);
			return true;
			};
		m_tree.query(fatAABB, treeCallback);
	}
	// 2. 排序并去重
	sort(m_pairBuffer.begin(), m_pairBuffer.end());
	m_pairBuffer.erase(std::unique(m_pairBuffer.begin(), m_pairBuffer.end()), m_pairBuffer.end());
	// 3. 触发外部回调
	for (const auto& pair : m_pairBuffer) {
		void* userDataA = m_tree.getUserData(pair.proxyIdA);
		void* userDataB = m_tree.getUserData(pair.proxyIdB);
		callback(userDataA, userDataB);
	}

	// 4. 清空移动缓冲区，准备下一帧
	m_moveBuffer.clear();
}

void BroadPhase::destroyProxy(int32_t proxyId) {
	m_tree.destroyProxy(proxyId);
	// 这里不做多余操作，直接删树里的
}

int32_t BroadPhase::createProxy(const AABB& aabb, void* userData) {
	// 1. 调用底层的树去创建
	int32_t proxyId = m_tree.createProxy(aabb, userData);

	// 2. 关键：把新创建的物体标记为“移动过”
	// 这样在接下来的 updatePairs 中，它会立即和树里的其他物体查一遍碰撞
	bufferMove(proxyId);

	return proxyId;
}
bool BroadPhase::moveProxy(int32_t proxyId, const AABB& aabb, const Vector2& displacement) {
	// 1. 调用底层的树去尝试移动
	bool changed = m_tree.moveProxy(proxyId, aabb, displacement);

	// 2. 如果树告诉我们：它跳出了肥包围盒，结构变了
	if (changed) {
		// 那就把它放进待查询名单
		bufferMove(proxyId);
	}
	return changed;
}

void BroadPhase::bufferMove(int32_t proxyId) {
	// 把移动过的物体索引放进 vector
	m_moveBuffer.push_back(proxyId);
}

bool BroadPhase::testOverlap(int32_t proxyIdA, int32_t proxyIdB) const
{
	const AABB& aabbA = m_tree.getFatAABB(proxyIdA);
	const AABB& aabbB = m_tree.getFatAABB(proxyIdB);

	// 调用你 AABB 结构体中已经写好的 overlap 函数
	return aabbA.overlap(aabbB);
}