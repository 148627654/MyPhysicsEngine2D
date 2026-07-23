#include "DynamicTree.h"
#include <iomanip> // 用于对齐输出
#include "../../include/physics/Collision/AABB.h"

#include <iostream>
#include <string>
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

int32_t DynamicTree::AllocateNode() {
	if (m_freelist == -1) {
		Node* oldNodes = m_nodes;
		int32_t oldCapacity = m_nodeCapacity;
		m_nodeCapacity *= 2;
		m_nodes = new Node[m_nodeCapacity];
		memcpy(m_nodes, oldNodes, oldCapacity * sizeof(Node));
		delete[] oldNodes;

		for (int32_t i = oldCapacity; i < m_nodeCapacity - 1; ++i) {
			m_nodes[i].next = i + 1;
			m_nodes[i].parent = -1;
		}
		m_nodes[m_nodeCapacity - 1].next = -1;
		m_freelist = oldCapacity; // 从第一个新柜子开始用
	}

	int32_t nodeId = m_freelist;
	m_freelist = m_nodes[nodeId].next;

	m_nodes[nodeId].parent = -1;
	m_nodes[nodeId].leftChild = -1;
	m_nodes[nodeId].rightChild = -1;
	m_nodes[nodeId].height = 0;
	m_nodes[nodeId].userData = nullptr;

	m_nodeCount++;
	return nodeId;
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
void DynamicTree::InsertLeaf(int32_t leaf) {
	if (m_root == -1) {
		m_root = leaf;
		m_nodes[m_root].parent = -1;
		return;
	}

	// 1. 寻找最佳兄弟
	AABB leafAABB = m_nodes[leaf].aabb;
	int32_t index = m_root;
	while (!m_nodes[index].isLeaf()) {
		int32_t child1 = m_nodes[index].leftChild;
		int32_t child2 = m_nodes[index].rightChild;

		// 这里的 cost 计算可以保留，但 Balance 必须给力
		float cost1 = AABB::Union(leafAABB, m_nodes[child1].aabb).GetPerimeter();
		float cost2 = AABB::Union(leafAABB, m_nodes[child2].aabb).GetPerimeter();
		index = (cost1 < cost2) ? child1 : child2;
	}
	int32_t sibling = index;

	// 2. 创建内部节点
	int32_t oldParent = m_nodes[sibling].parent;
	int32_t newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = nullptr;
	m_nodes[newParent].aabb = AABB::Union(leafAABB, m_nodes[sibling].aabb);
	m_nodes[newParent].height = m_nodes[sibling].height + 1;

	if (oldParent != -1) {
		if (m_nodes[oldParent].leftChild == sibling) m_nodes[oldParent].leftChild = newParent;
		else m_nodes[oldParent].rightChild = newParent;
	}
	else {
		m_root = newParent;
	}
	m_nodes[newParent].leftChild = sibling;
	m_nodes[newParent].rightChild = leaf;
	m_nodes[sibling].parent = newParent;
	m_nodes[leaf].parent = newParent;

	// 3. 向上回溯更新并平衡
	int32_t p = m_nodes[leaf].parent;
	while (p != -1) {
		UpdateNodeMetadata(p);

		p = Balance(p); // Balance 内部必须先执行 UpdateNodeMetadata

		// 旋转后 p 已经指向了该子树的新根节点，向上爬
		p = m_nodes[p].parent;
	}
}

int32_t DynamicTree::Balance(int32_t iA) {
	if (m_nodes[iA].isLeaf() || m_nodes[iA].height < 2) return iA;

	int32_t iB = m_nodes[iA].leftChild;
	int32_t iC = m_nodes[iA].rightChild;
	int32_t balance = m_nodes[iC].height - m_nodes[iB].height;

	// 右子树太深 -> 把 C 提上来
	if (balance > 1) {
		int32_t iF = m_nodes[iC].leftChild;
		int32_t iG = m_nodes[iC].rightChild;

		// C 提拔为 A 的父节点位置
		m_nodes[iC].leftChild = iA;
		m_nodes[iC].parent = m_nodes[iA].parent;
		m_nodes[iA].parent = iC;

		// 处理爷爷节点的指向
		if (m_nodes[iC].parent != -1) {
			if (m_nodes[m_nodes[iC].parent].leftChild == iA) m_nodes[m_nodes[iC].parent].leftChild = iC;
			else m_nodes[m_nodes[iC].parent].rightChild = iC;
		}
		else {
			m_root = iC;
		}

		// 核心旋转：比较 C 的子节点 F 和 G 哪个更高
		if (m_nodes[iF].height > m_nodes[iG].height) {
			// RL: C 原本的右儿子 G 换给 A 当右儿子
			m_nodes[iC].rightChild = iF;
			m_nodes[iA].rightChild = iG;
			m_nodes[iG].parent = iA;
		}
		else {
			// RR: C 原本的左儿子 F 换给 A 当右儿子
			m_nodes[iC].rightChild = iG;
			m_nodes[iA].rightChild = iF;
			m_nodes[iF].parent = iA;
		}

		// 更新高度和 AABB (必须先更 A，再更 C)
		UpdateNodeMetadata(iA);
		UpdateNodeMetadata(iC);
		return iC;
	}

	// 左子树太深 -> 把 B 提上来
	if (balance < -1) {
		int32_t iF = m_nodes[iB].leftChild;
		int32_t iG = m_nodes[iB].rightChild;

		m_nodes[iB].rightChild = iA;
		m_nodes[iB].parent = m_nodes[iA].parent;
		m_nodes[iA].parent = iB;

		if (m_nodes[iB].parent != -1) {
			if (m_nodes[m_nodes[iB].parent].leftChild == iA) m_nodes[m_nodes[iB].parent].leftChild = iB;
			else m_nodes[m_nodes[iB].parent].rightChild = iB;
		}
		else {
			m_root = iB;
		}

		if (m_nodes[iF].height > m_nodes[iG].height) {
			// LL: B 原本的右儿子 G 换给 A 当左儿子
			m_nodes[iB].leftChild = iF;
			m_nodes[iA].leftChild = iG;
			m_nodes[iG].parent = iA;
		}
		else {
			// LR: B 原本的左儿子 F 换给 A 当左儿子
			m_nodes[iB].leftChild = iG;
			m_nodes[iA].leftChild = iF;
			m_nodes[iF].parent = iA;
		}

		UpdateNodeMetadata(iA);
		UpdateNodeMetadata(iB);
		return iB;
	}

	return iA;
}

// 辅助函数：统一更新 AABB 和高度
void DynamicTree::UpdateNodeMetadata(int32_t i) {
	int32_t c1 = m_nodes[i].leftChild;
	int32_t c2 = m_nodes[i].rightChild;
	m_nodes[i].height = 1 + std::max(m_nodes[c1].height, m_nodes[c2].height);
	m_nodes[i].aabb = AABB::Union(m_nodes[c1].aabb, m_nodes[c2].aabb);
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
	RemoveLeaf(proxyId);
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




void DynamicTree::Describe() const {
	if (m_root == -1) {
		std::cout << "--- Tree is Empty ---" << std::endl;
		return;
	}
	std::cout << "--- Tree Hierarchy (Root: " << m_root << ") ---" << std::endl;
	DescribeNode(m_root, 0);
	std::cout << "------------------------------------------" << std::endl;
}

void DynamicTree::DescribeNode(int32_t nodeId, int32_t depth) const {
	if (nodeId == -1) return;

	const Node& node = m_nodes[nodeId];

	// 生成缩进
	std::string indent = "";
	for (int i = 0; i < depth; ++i) {
		indent += (i == depth - 1) ? "|--" : "   ";
	}

	// 打印当前节点信息
	std::cout << indent << "[Slot " << nodeId << "] ";
	if (node.isLeaf()) {
		Body* body = static_cast<Body*>(node.userData);
		float posX = body ? body->GetPosition().getX() : -1.0f;
		std::cout << "LEAF (BodyX: " << posX << ")";
	}
	else {
		std::cout << "INTERNAL (Height: " << node.height
			<< ", X: " << node.aabb.min.getX() << " to " << node.aabb.max.getX() << ")";
	}
	std::cout << std::endl;

	// 递归打印子节点
	if (!node.isLeaf()) {
		DescribeNode(node.leftChild, depth + 1);
		DescribeNode(node.rightChild, depth + 1);
	}
}

void DynamicTree::RemoveLeaf(int32_t leafId) {
	if (leafId == m_root) {
		m_root = -1;
		return;
	}

	int32_t parent = m_nodes[leafId].parent;
	int32_t grandParent = m_nodes[parent].parent;

	// 找到兄弟节点
	int32_t sibling = (m_nodes[parent].leftChild == leafId) ?
		m_nodes[parent].rightChild : m_nodes[parent].leftChild;

	if (grandParent != -1) {
		// 1. 让爷爷指向兄弟，用兄弟替换掉原来的父节点
		if (m_nodes[grandParent].leftChild == parent) {
			m_nodes[grandParent].leftChild = sibling;
		}
		else {
			m_nodes[grandParent].rightChild = sibling;
		}
		m_nodes[sibling].parent = grandParent;

		// 2. 释放掉多余的内部节点
		FreeNode(parent);

		// 3. 从爷爷开始向上回溯更新并平衡
		int32_t p = grandParent;
		while (p != -1) {
			UpdateNodeMetadata(p);
			p = Balance(p);
			p = m_nodes[p].parent;
		}
	}
	else {
		// 父节点就是根节点，直接让兄弟变成新根
		m_root = sibling;
		m_nodes[sibling].parent = -1;
		FreeNode(parent);
	}
}
