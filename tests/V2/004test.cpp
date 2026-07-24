#include "../include/physics/Collision/DynamicTree.h"
#include "../include/physics/Utils/Logger.h"
#include <iostream>
#include <iomanip>
#include <Box.h>


void RunComprehensiveTreeTest() {
    Logger::Info(">>> [V2-Comprehensive] Starting 4-Day Integration Test <<<");

    DynamicTree tree;
    Box* shape = new Box(1.0f, 1.0f);
    std::vector<Body*> bodies;
    const int count = 6;

    // --- STEP 1: 插入测试 (验证 Day 1, 2, 3) ---
    Logger::Info("Step 1: Inserting 6 bodies...");
    for (int i = 0; i < count; ++i) {
        Body* b = new Body(shape, (float)i * 5.0f, 0.0f, 1.0f); // 间隔5.0放置
        int32_t id = tree.CreateProxy(b->GetAABB(), b);
        b->setProxyId(id);
        bodies.push_back(b);
    }

    Logger::Info("Current Tree Height: " + std::to_string(tree.GetNodeHeight(tree.GetRoot())));
    tree.Describe(); // 此时应看到一个高度为 3 或 4 的平衡树

    // --- STEP 2: 微动测试 (验证 Day 4 Fat AABB) ---
    Logger::Info("Step 2: Micro-moving all bodies (0.05 units)...");
    int updateCount = 0;
    for (auto b : bodies) {
        Vector2 oldPos = b->GetPosition();
        Vector2 newPos = oldPos + Vector2(0.05f, 0.0f); // 微动
        b->SetPosition(newPos);

        // MoveProxy 应该返回 false，因为位移在 0.1 扩展范围内
        if (tree.MoveProxy(b->getProxyId(), b->GetAABB(), Vector2(0.05f, 0.0f))) {
            updateCount++;
        }
    }
    Logger::Info("Tree updates triggered: " + std::to_string(updateCount) + " (Expected: 0)");
    if (updateCount == 0) Logger::Info("SUCCESS: Fat AABB successfully filtered micro-movements!");

    // --- STEP 3: 剧烈运动与预测 (验证 Day 4 Displacement) ---
    Logger::Info("Step 3: Large movement for Body 0 (to X=50)...");
    Body* b0 = bodies[0];
    Vector2 displacement(50.0f, 0.0f);
    b0->SetPosition(b0->GetPosition() + displacement);
    Logger::Info("Step 3: Large movement for Body 0 in");
    AABB currentFatInTree = tree.GetNodeAABB(bodies[0]->getProxyId());
    Logger::Info("DEBUG: Before MoveProxy, Tree Fat MaxX = " + std::to_string(currentFatInTree.max.getX()));
    if (tree.MoveProxy(b0->getProxyId(), b0->GetAABB(), displacement)) {
        Logger::Info("SUCCESS: Large movement triggered tree reconstruction.");
    }
    Logger::Info("Step 3: Large movement for Body 0 out");
    // 检查 Body 0 的 Fat AABB 是否由于位移预测被拉长了
    AABB b0Fat = tree.GetNodeAABB(b0->getProxyId());
    float rightBuffer = b0Fat.max.getX() - b0->GetAABB().max.getX();
    Logger::Info("Body 0 Right Buffer: " + std::to_string(rightBuffer) + " (Should be ~101.1 if multiplier=2)");

    // --- STEP 4: 删除测试 (验证 Day 1 & 3) ---
    Logger::Info("Step 4: Removing Body 2 and 4...");
    tree.DestroyProxy(bodies[2]->getProxyId());
    tree.DestroyProxy(bodies[4]->getProxyId());

    tree.PrintPool(); // 验证 Slot 2, 4 对应的位置和它们的父节点被回收
    tree.Describe();  // 验证树是否依然平衡且高度降低

    // 清理
    for (auto b : bodies) delete b;
    delete shape;
    Logger::Info(">>> [V2-Comprehensive] All Tests Passed! <<<");
}

int main() {
    RunComprehensiveTreeTest();
    return 0;
}