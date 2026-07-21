#include "DynamicTree.h"
#include <iomanip> // 用于对齐输出
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

int32_t DynamicTree::CreateProxy(const AABB& aabb, void* userData) {
	// 拿一个空柜子
	int32_t proxyId = AllocateNode();

	// 设置叶子节点的信息
	m_nodes[proxyId].aabb = aabb;
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].height = 0;

	// --- 注意：Day 1 只需要到这里 ---
	// InsertLeaf(proxyId); 

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