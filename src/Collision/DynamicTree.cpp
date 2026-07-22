#include "DynamicTree.h"
#include <iomanip> // 用于对齐输出
#include "../../include/physics/Collision/AABB.h"
DynamicTree::DynamicTree()
{
	m_nodeCapacity = 16;//初始化容量
	m_nodeCount = 0;
	m_nodes = new Node[m_nodeCapacity];
	m_root = -1;

	for (int32_t i = 0;i < m_nodeCapacity - 1;++i)
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].parent = -1;
	}
	m_nodes[m_nodeCapacity - 1].next = -1;
	m_freelist = 0;
}

DynamicTree::~DynamicTree()
{
	delete[]m_nodes;
}

int32_t DynamicTree::AllocateNode()
{
	if (m_freelist == -1)
	{
		Node* oldNode = m_nodes;
		m_nodeCapacity *= 2;
		m_nodes = new Node[m_nodeCapacity];
		memcpy(m_nodes, oldNode, m_nodeCount * sizeof(Node));
		delete[]oldNode;
		for (int32_t i = 0;i < m_nodeCapacity - 1;++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].parent = -1;
		}
		m_nodes[m_nodeCapacity - 1].next = -1;
		m_freelist = m_nodeCount;
	}
	

	int32_t NodeID = m_freelist;
	m_freelist = m_nodes[NodeID].next;
	//初始化m_freelist
	m_nodes[NodeID].parent = -1;
	m_nodes[NodeID].leftChild = -1;
	m_nodes[NodeID].rightChild = -1;
	m_nodes[NodeID].height = 0;
	m_nodes[NodeID].userData = nullptr;

	m_nodeCount++;
	return NodeID;
}

void DynamicTree::FreeNode(int32_t nodeId)
{
	m_nodes[nodeId].next = m_freelist;
	m_nodes[nodeId].parent = -1;

	m_freelist = nodeId;
	m_nodeCount--;
}

/*
1.处理空树
2.寻找最佳兄弟：从根节点开始遍历，计算插入当前分支的“代价”。
	简单版*：寻找合并后 AABB 周长增加最小的节点。
3.建立连接：
	分配内部节点。
	修改原兄弟节点的父节点指向。
	设置新内部节点的 `child1`, `child2`, `parent`。
4.回溯更新：循环向上执行 `m_nodes[p].aabb = Union(...)`。
*/
void DynamicTree::InsertLeaf(int32_t leaf)
{
	// 1. 处理空树
	if (m_root == -1)
	{
		m_root = leaf;
		m_nodes[m_root].parent = -1;
		return;
	}

	// 2. 寻找最佳兄弟 (目前这一步你的逻辑基本是对的)
	AABB leafAABB = m_nodes[leaf].aabb;
	int32_t index = m_root;
	while (!m_nodes[index].isLeaf())
	{
		int32_t child1 = m_nodes[index].leftChild;
		int32_t child2 = m_nodes[index].rightChild;

		float cost1 = AABB::Union(leafAABB, m_nodes[child1].aabb).GetPerimeter();
		float cost2 = AABB::Union(leafAABB, m_nodes[child2].aabb).GetPerimeter();

		index = (cost1 < cost2) ? child1 : child2;
	}
	int32_t sibling = index;

	// 3. 创建新的内部节点作为父节点
	int32_t oldParent = m_nodes[sibling].parent;
	int32_t newParent = AllocateNode(); // 【重点】这里会增加 nodeCount!

	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr; // 内部节点必须是空
	m_nodes[newParent].aabb = AABB::Union(leafAABB, m_nodes[sibling].aabb);
	m_nodes[newParent].height = m_nodes[sibling].height + 1;

	// 4. 处理连接关系
	if (oldParent != -1)
	{
		// 原本指向兄弟的索引，现在指向新爸爸
		if (m_nodes[oldParent].leftChild == sibling)
			m_nodes[oldParent].leftChild = newParent;
		else
			m_nodes[oldParent].rightChild = newParent;
	}
	else
	{
		// 【关键修正】：如果兄弟原本是根，现在新爸爸上位
		m_root = newParent;
	}

	m_nodes[newParent].leftChild = sibling;
	m_nodes[newParent].rightChild = leaf;
	m_nodes[sibling].parent = newParent;
	m_nodes[leaf].parent = newParent;

	// 5. 回溯更新 AABB
	int32_t p = m_nodes[leaf].parent;
	while (p != -1)
	{
		int32_t c1 = m_nodes[p].leftChild;
		int32_t c2 = m_nodes[p].rightChild;

		m_nodes[p].aabb = AABB::Union(m_nodes[c1].aabb, m_nodes[c2].aabb);
		m_nodes[p].height = 1 + std::max(m_nodes[c1].height, m_nodes[c2].height);

		p = m_nodes[p].parent;
	}
}

int32_t DynamicTree::CreateProxy(const AABB& aabb, void* userData) {
	// 拿一个空柜子
	int32_t proxyId = AllocateNode();

	// 设置叶子节点的信息
	m_nodes[proxyId].aabb = aabb;
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].height = 0;

	// --- 注意：Day 1 只需要到这里 ---
	 InsertLeaf(proxyId); 

	return proxyId;
}

void DynamicTree::DestroyProxy(int32_t proxyId) {
	// 逻辑：
	// 1. 从树中移除该节点 (RemoveLeaf)
	// 2. 释放节点
	FreeNode(proxyId);
}



void DynamicTree::PrintPool() const {
	Logger::Info("=== DynamicTree Node Pool State ===");

	// 基础信息打印
	std::cout << "Capacity: " << m_nodeCapacity
		<< " | Active Count: " << m_nodeCount
		<< " | FreeList Head: " << m_freelist << std::endl;

	// 1. 快速标记哪些节点是空闲的 (为了打印准确)
	// 逻辑：遍历一次空闲链表，把索引存在一个 bool 数组里
	std::vector<bool> isFreeMap(m_nodeCapacity, false);
	int32_t nextFree = m_freelist;
	while (nextFree != -1) {
		isFreeMap[nextFree] = true;
		nextFree = m_nodes[nextFree].next;
	}

	// 2. 遍历整个数组并打印
	for (int32_t i = 0; i < m_nodeCapacity; ++i) {
		const Node& n = m_nodes[i];

		// 打印槽位编号
		std::cout << "[Slot " << std::setw(2) << i << "] ";

		if (isFreeMap[i]) {
			// --- 情况 A: 空闲节点 ---
			// 打印白色，显示链表中的下一个位置
			std::cout << "TYPE: FREE   | NEXT: " << std::setw(2) << n.next << std::endl;
		}
		else {
			// --- 情况 B: 活动节点 ---
			// 使用 ANSI 绿色加亮 (\033[32m)
			std::cout << "\033[32m" << "TYPE: ACTIVE | ";

			if (n.userData != nullptr) {
				// 强制转换为 Body* 指针并读取位置
				Body* body = static_cast<Body*>(n.userData);
				std::cout << "BODY POS: ("
					<< std::setw(5) << std::fixed << std::setprecision(1) << body->GetPosition().getX() << ", "
					<< std::setw(5) << std::fixed << std::setprecision(1) << body->GetPosition().getY() << ")";
			}
			else {
				// 如果是内部节点（Day 2 会用到），userData 可能为空
				std::cout << "USERDATA: nullptr";
			}

			// 恢复颜色 (\033[0m)
			std::cout << "\033[0m" << std::endl;
		}
	}
	Logger::Info("====================================");
}

void DynamicTree::Validate() const {
	if (m_root == -1) return;

	Logger::Info("--- Tree Structural Validation ---");
	for (int32_t i = 0; i < m_nodeCapacity; ++i) {
		const Node& n = m_nodes[i];
		if (n.parent == -1 && i != m_root) continue; // 跳过空位

		if (!n.isLeaf()) {
			// 验证内部节点的 AABB 是否包裹了子节点
			AABB childUnion = AABB::Union(m_nodes[n.leftChild].aabb, m_nodes[n.rightChild].aabb);
			// 这里可以比较 min/max，如果不相等说明回溯更新失败
			if (n.aabb.min.getX() > childUnion.min.getX() || n.aabb.max.getX() < childUnion.max.getX()) {
				Logger::Error("AABB MISMATCH at Slot " + std::to_string(i));
			}

			// 验证双向连接
			if (m_nodes[n.leftChild].parent != i || m_nodes[n.rightChild].parent != i) {
				Logger::Error("PARENT-CHILD LINK BROKEN at Slot " + std::to_string(i));
			}
		}
	}
	Logger::Info("Validation Finished. Root is Slot: " + std::to_string(m_root));
}
