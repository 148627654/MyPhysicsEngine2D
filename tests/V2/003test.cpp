#include "../include/physics/Collision/DynamicTree.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Utils/Logger.h"
#include <vector>

void RunDay3RotationTest() {
    Logger::info("Starting V2-Day 3: Tree Rotation & balance Test");

    DynamicTree tree;
    Box* shape = new Box(1.0f, 1.0f);
    const int objectCount = 12;
    std::vector<Body*> bodies;

    Logger::info("Action: Inserting " + std::to_string(objectCount) + " bodies...");

    for (int i = 0; i < objectCount; ++i) {
        Body* b = new Body(shape, (float)i * 2.0f, 0.0f, 1.0f);
        int32_t proxyId = tree.createProxy(b->getAABB(), b);
        b->setProxyId(proxyId);
        bodies.push_back(b);
    }

    Logger::info("Initial Tree State:");
    tree.describe();
    Logger::info("Initial Height: " + std::to_string(tree.getNodeHeight(tree.getRoot())));

    // --- 新增：删除测试 ---
    Logger::info("Action: Removing odd-indexed bodies (1, 3, 5, 7, 9, 11)...");
    for (int i = 1; i < objectCount; i += 2) {
        tree.destroyProxy(bodies[i]->getProxyId());
    }

    // 验证节点总数
    // 剩余 6 个叶子，应该有 5 个内部节点，总计 11 个 Active 节点
    tree.printPool();

    // 验证剩余树的高度
    int32_t rootId = tree.getRoot();
    int treeHeight = tree.getNodeHeight(rootId);
    Logger::info("Tree Height after removal: " + std::to_string(treeHeight));

    if (treeHeight <= 4) {
        Logger::info("SUCCESS: Tree remains balanced after removals!");
    }
    else {
        Logger::error("FAILURE: Tree became unbalanced after removals.");
    }

    // 验证 AABB 是否缩减或依然有效
    // 剩余物体：0, 2, 4, 6, 8, 10
    // 范围大约是 -0.5 到 10.5
    const AABB& rootAABB = tree.getNodeAABB(rootId);
    Logger::info("Root AABB after removal: Min(" + std::to_string(rootAABB.min.getX()) + "), Max(" + std::to_string(rootAABB.max.getX()) + ")");

    if (rootAABB.max.getX() < 22.0f) {
        Logger::info("SUCCESS: Root AABB shrunk correctly.");
    }
    else {
        Logger::error("FAILURE: Root AABB did not shrink after removals!");
    }

    tree.describe();

    // 清理内存
    for (auto b : bodies) delete b;
    delete shape;

    Logger::info("V2-Day 3 Test Completed.");
}

//int main()
//{
//    RunDay3RotationTest();
//    return 0;
//}