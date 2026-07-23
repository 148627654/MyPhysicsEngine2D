#pragma once
#include "AABB.h"
#include "../Utils/Logger.h"
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

	int32_t CreateProxy(const AABB& aabb , void* userData);
	void DestroyProxy(int32_t proxyId);
	void PrintPool( ) const;
	inline int32_t GetRoot()const { return m_root; }
	inline const AABB& GetNodeAABB(int32_t nodeId) const { return m_nodes[nodeId].aabb; }
	inline int32_t GetNodeHeight(int32_t rootId)const { if (rootId == -1) return 0;
	return m_nodes[rootId].height; }
	void Describe() const; // 打印树状结构
private:
	int32_t AllocateNode( );
	void FreeNode(int32_t nodeId);
	void InsertLeaf(int32_t leaf);
	int32_t Balance(int32_t iA);//LL,RR,LR,RL
	void UpdateNodeMetadata(int32_t i);
	void DescribeNode(int32_t nodeId, int32_t depth) const; // 递归辅助
	void RemoveLeaf(int32_t leafId);

	Node* m_nodes;
	int32_t m_nodeCount;
	int32_t m_nodeCapacity;
	int32_t m_root;
	int32_t m_freelist;
};