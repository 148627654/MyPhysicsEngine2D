//#include "../include/physics/Collision/DynamicTree.h"
//#include "../include/physics/Collision/Box.h"
//#include "../include/physics/Dynamics/Body.h"
//#include "../include/physics/Utils/Logger.h"
//
//void RunRealBodyTreeTest() {
//    Logger::info("Starting Real Body Tree Integration Test...");
//    DynamicTree tree;
//    Box* shape = new Box(1.0f, 1.0f);
//
//    // 1. 创建 3 个真实的 Body
//    Body* bodyA = new Body(shape, 10.0f, 20.0f, 1.0f); // 坐标 (10, 20)
//    Body* bodyB = new Body(shape, -5.0f, 0.0f, 1.0f);  // 坐标 (-5, 0)
//    Body* bodyC = new Body(shape, 100.0f, 100.0f, 0.0f); // 静态 (100, 100)
//
//    // 2. 将 Body 注册到树中
//    Logger::info("Action: Registering Bodies into DynamicTree...");
//    bodyA->setProxyId(tree.createProxy(bodyA->getAABB(), bodyA));
//    bodyB->setProxyId(tree.createProxy(bodyB->getAABB(), bodyB));
//    bodyC->setProxyId(tree.createProxy(bodyC->getAABB(), bodyC));
//
//    // 3. 观察输出
//    tree.printPool();
//
//    // 4. 模拟 BodyB 移动并更新树
//    Logger::info("Action: Moving BodyB and updating proxy...");
//    bodyB->setPosition(Vector2(0.0f, 0.0f));
//    bodyB->updateAABB();
//
//    // 在 V2-Day 4 我们会写动态更新，现在我们先手动模拟删除再创建
//    tree.destroyProxy(bodyB->getProxyId());
//    bodyB->setProxyId(tree.createProxy(bodyB->getAABB(), bodyB));
//
//    tree.printPool();
//
//    Logger::info("V2-Day 1 Integration Test Finished.");
//
//    // 清理
//    delete shape; delete bodyA; delete bodyB; delete bodyC;
//}
//
//int main()
//{
//    RunRealBodyTreeTest();
//    return 0;
//}