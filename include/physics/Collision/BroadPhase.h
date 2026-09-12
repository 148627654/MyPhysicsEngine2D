#pragma once
#include "DynamicTree.h"
struct Pair
{
	int32_t proxyIdA;
	int32_t proxyIdB;

	bool operator<(const Pair& other)
	{
		if (proxyIdA < other.proxyIdA) return true;
		if (proxyIdA == other.proxyIdA) return proxyIdB < other.proxyIdB;
		return false;
	}

	bool operator==(const Pair& other) const {
		return proxyIdA == other.proxyIdA && proxyIdB == other.proxyIdB;
	}
};

using BroadPhaseCallback = std::function<void(void* userDataA, void* userDataB)>;

class BroadPhase
{
public:
	void UpdatePairs(BroadPhaseCallback callback);
	int32_t CreateProxy(const AABB& aabb, void* userData);
	bool MoveProxy(int32_t proxyId, const AABB& aabb, const Vector2& displacement);
	void DestroyProxy(int32_t proxyId);
	void RayCast(RayCastInput& input, std::function<float(RayCastInput&, int32_t)> callback) {
		m_tree.RayCast(input, callback);
	}
	bool TestOverlap(int32_t proxyIdA, int32_t proxyIdB) const;
	void* GetUserData(int32_t proxyId) const {
		return m_tree.GetUserData(proxyId);
	}
	int m_moveCount = 0; // 记录 MoveProxy 实际执行了多少次重构
private:
	void BufferMove(int32_t proxyId);
	DynamicTree m_tree;
	std::vector<int32_t> m_moveBuffer;				//(存储本帧移动过的物体索引)。
	std::vector<Pair> m_pairBuffer;					//(存储发现的潜在碰撞对)。
};