#include "../include/physics/Collision/BroadPhase.h"
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Utils/Logger.h"
#include <vector>
#include <iomanip> // 用于对齐打印

void RunDay5BroadPhaseTestDetailed() {
    Logger::Info(">>> Starting V2-Day 5: BroadPhase Detailed Integration Test <<<");

    BroadPhase broadPhase;
    Box shape(1.0f, 1.0f);
    std::vector<Body*> bodies;

    // --- 增强版回调函数 ---
    // 不仅计数，还打印出具体的碰撞信息
    int totalPairsFound = 0;
    auto detailedCallback = [&](void* ua, void* ub) {
        Body* ba = static_cast<Body*>(ua);
        Body* bb = static_cast<Body*>(ub);
        totalPairsFound++;

        std::cout << "    [MATCH] Potential Collision #" << totalPairsFound << ": "
            << "Body(ID:" << ba->getProxyId() << ", Pos:" << ba->GetPosition().getX() << ") <--> "
            << "Body(ID:" << bb->getProxyId() << ", Pos:" << bb->GetPosition().getX() << ")" << std::endl;
        };

    // --- 1. 创建物体并观察 ID 分配 ---
    Logger::Info("Step 1: Initializing World State...");

    // B0 at (0,0), B1 at (0.5, 0) -> 应该重叠
    Body* b0 = new Body(&shape, 0.0f, 0.0f);
    b0->setProxyId(broadPhase.CreateProxy(b0->GetAABB(), b0));

    Body* b1 = new Body(&shape, 0.5f, 0.0f);
    b1->setProxyId(broadPhase.CreateProxy(b1->GetAABB(), b1));

    // B2 放在很远的地方 (10, 10)
    Body* b2 = new Body(&shape, 10.0f, 10.0f);
    b2->setProxyId(broadPhase.CreateProxy(b2->GetAABB(), b2));

    bodies.push_back(b0); bodies.push_back(b1); bodies.push_back(b2);

    std::cout << "  - Body 0 assigned ProxyID: " << b0->getProxyId() << " at (0.0, 0.0)" << std::endl;
    std::cout << "  - Body 1 assigned ProxyID: " << b1->getProxyId() << " at (0.5, 0.0)" << std::endl;
    std::cout << "  - Body 2 assigned ProxyID: " << b2->getProxyId() << " at (10.0, 10.0)" << std::endl;

    totalPairsFound = 0;
    Logger::Info("BroadPhase: Running UpdatePairs (Initial check)...");
    broadPhase.UpdatePairs(detailedCallback);
    Logger::Info("Result: Found " + std::to_string(totalPairsFound) + " pairs. (Expected: 1 [B0-B1])");

    // --- 2. 模拟物体的微小抖动 (验证性能优化) ---
    Logger::Info("Step 2: Jiggling Body 2 (Micro-move 0.02 units)...");
    Vector2 microDisplacement(0.02f, 0.02f);
    b2->SetPosition(b2->GetPosition() + microDisplacement);

    // MoveProxy 返回 false 表示物体仍在肥包围盒内，不需要重构
    bool wasReconstructed = broadPhase.MoveProxy(b2->getProxyId(), b2->GetAABB(), microDisplacement);

    std::cout << "  - MoveProxy report: " << (wasReconstructed ? "RECONSTRUCTED (Tree Changed)" : "STABLE (Still inside Fat AABB)") << std::endl;

    totalPairsFound = 0;
    broadPhase.UpdatePairs(detailedCallback);
    Logger::Info("Result: Found " + std::to_string(totalPairsFound) + " pairs. (Expected: 0)");

    // --- 3. 模拟剧烈运动进入碰撞区 ---
    Logger::Info("Step 3: Moving Body 2 from (10,10) to (0.2, 0.2)...");
    Vector2 oldPos = b2->GetPosition();
    Vector2 newPos(0.2f, 0.2f);
    b2->SetPosition(newPos);

    wasReconstructed = broadPhase.MoveProxy(b2->getProxyId(), b2->GetAABB(), newPos - oldPos);
    std::cout << "  - MoveProxy report: " << (wasReconstructed ? "RECONSTRUCTED" : "STABLE") << std::endl;

    totalPairsFound = 0;
    broadPhase.UpdatePairs(detailedCallback);
    Logger::Info("Result: Found " + std::to_string(totalPairsFound) + " pairs. (Expected: 2 [B2-B0, B2-B1])");

    // --- 4. 清理 ---
    Logger::Info("Step 4: Destruction Test...");
    for (size_t i = 0; i < bodies.size(); ++i) {
        std::cout << "  - Destroying ProxyID: " << bodies[i]->getProxyId() << std::endl;
        broadPhase.DestroyProxy(bodies[i]->getProxyId());
        delete bodies[i];
    }

    Logger::Info(">>> V2-Day 5 Detailed Test Completed Successfully. <<<");
}
//int main() {
//    RunDay5BroadPhaseTestDetailed();
//    return 0;
//}