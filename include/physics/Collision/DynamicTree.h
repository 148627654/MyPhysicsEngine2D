#pragma once
#include "AABB.h"
#include "../Utils/Logger.h"
#include <functional>
#include <assert.h>



struct Node
{
	AABB aabb;
	void* userData; // 存储 Body*

	int32_t parent;
	int32_t leftChild;
	int32_t rightChild;

	//用于空闲列表
	union {
		int32_t next;
		int32_t height;			//平衡树
	};

	bool isLeaf()const { return leftChild == -1; }
};

class DynamicTree
{
public:
	DynamicTree( );
	~DynamicTree( );

	int32_t createProxy(const AABB& aabb , void* userData);
	void destroyProxy(int32_t proxyId);
	void printPool( ) const;
	inline int32_t getRoot()const { return m_root; }
	inline const AABB& getNodeAABB(int32_t nodeId) const { return m_nodes[nodeId].aabb; }
	inline int32_t getNodeHeight(int32_t rootId)const { if (rootId == -1) return 0;
	return m_nodes[rootId].height; }
	void describe() const; // 打印树状结构
	bool moveProxy(int32_t proxyId, const AABB& aabb, const Vector2& displacement);
	void query(const AABB& aabb, std::function<bool(int32_t)> callback);
	inline void* getUserData(int32_t proxyId) const {
		assert(proxyId >= 0 && proxyId < m_nodeCapacity); 
		return m_nodes[proxyId].userData;
	}
	inline bool isLeaf(int32_t nodeId) const {
		// 安全检查：索引必须在合法范围内
		if (nodeId < 0 || nodeId >= m_nodeCapacity) return false;
		return m_nodes[nodeId].isLeaf();
	}
	const AABB& getFatAABB(int32_t proxyId) const {
		// 假设你的节点数组叫 m_nodes
		return m_nodes[proxyId].aabb;
	}
	void rayCast(RayCastInput& input, std::function<float(RayCastInput& input, int32_t nodeId)> callback);
private:
	int32_t allocateNode( );
	void freeNode(int32_t nodeId);
	void insertLeaf(int32_t leaf);
	int32_t balance(int32_t iA);//LL,RR,LR,RL
	void updateNodeMetadata(int32_t i);
	void describeNode(int32_t nodeId, int32_t depth) const; // 递归辅助
	void removeLeaf(int32_t leafId);
	
	Node* m_nodes;
	int32_t m_nodeCount;
	int32_t m_nodeCapacity;
	int32_t m_root;
	int32_t m_freelist;
};

