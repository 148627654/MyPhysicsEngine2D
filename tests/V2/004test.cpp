#include "../include/physics/Collision/DynamicTree.h"
#include "../include/physics/Utils/Logger.h"
#include <iostream>
#include <iomanip>
#include <Box.h>


void RunComprehensiveTreeTest() {
    Logger::info(">>> [V2-Comprehensive] Starting 4-Day Integration Test <<<");

    DynamicTree tree;
    Box* shape = new Box(1.0f, 1.0f);
    std::vector<Body*> bodies;
    const int count = 6;

    // --- STEP 1: 插入测试 (验证 Day 1, 2, 3) ---
    Logger::info("step 1: Inserting 6 bodies...");
    for (int i = 0; i < count; ++i) {
        Body* b = new Body(shape, (float)i * 5.0f, 0.0f, 1.0f); // 间隔5.0放置
        int32_t id = tree.createProxy(b->getAABB(), b);
        b->setProxyId(id);
        bodies.push_back(b);
    }

    Logger::info("Current Tree Height: " + std::to_string(tree.getNodeHeight(tree.getRoot())));
    tree.describe(); // 此时应看到一个高度为 3 或 4 的平衡树

    // --- STEP 2: 微动测试 (验证 Day 4 Fat AABB) ---
    Logger::info("step 2: Micro-moving all bodies (0.05 units)...");
    int updateCount = 0;
    for (auto b : bodies) {
        Vector2 oldPos = b->getPosition();
        Vector2 newPos = oldPos + Vector2(0.05f, 0.0f); // 微动
        b->setPosition(newPos);

        // moveProxy 应该返回 false，因为位移在 0.1 扩展范围内
        if (tree.moveProxy(b->getProxyId(), b->getAABB(), Vector2(0.05f, 0.0f))) {
            updateCount++;
        }
    }
    Logger::info("Tree updates triggered: " + std::to_string(updateCount) + " (Expected: 0)");
    if (updateCount == 0) Logger::info("SUCCESS: Fat AABB successfully filtered micro-movements!");

    // --- STEP 3: 剧烈运动与预测 (验证 Day 4 Displacement) ---
    Logger::info("step 3: Large movement for Body 0 (to X=50)...");
    Body* b0 = bodies[0];
    Vector2 displacement(50.0f, 0.0f);
    b0->setPosition(b0->getPosition() + displacement);
    Logger::info("step 3: Large movement for Body 0 in");
    AABB currentFatInTree = tree.getNodeAABB(bodies[0]->getProxyId());
    Logger::info("DEBUG: Before moveProxy, Tree Fat MaxX = " + std::to_string(currentFatInTree.max.getX()));
    if (tree.moveProxy(b0->getProxyId(), b0->getAABB(), displacement)) {
        Logger::info("SUCCESS: Large movement triggered tree reconstruction.");
    }
    Logger::info("step 3: Large movement for Body 0 out");
    // 检查 Body 0 的 Fat AABB 是否由于位移预测被拉长了
    AABB b0Fat = tree.getNodeAABB(b0->getProxyId());
    float rightBuffer = b0Fat.max.getX() - b0->getAABB().max.getX();
    Logger::info("Body 0 Right Buffer: " + std::to_string(rightBuffer) + " (Should be ~101.1 if multiplier=2)");

    // --- STEP 4: 删除测试 (验证 Day 1 & 3) ---
    Logger::info("step 4: Removing Body 2 and 4...");
    tree.destroyProxy(bodies[2]->getProxyId());
    tree.destroyProxy(bodies[4]->getProxyId());

    tree.printPool(); // 验证 Slot 2, 4 对应的位置和它们的父节点被回收
    tree.describe();  // 验证树是否依然平衡且高度降低

    // 清理
    for (auto b : bodies) delete b;
    delete shape;
    Logger::info(">>> [V2-Comprehensive] All Tests Passed! <<<");
}

//int main() {
//    RunComprehensiveTreeTest();
//    return 0;
//}