//#include "../include/physics/Collision/DynamicTree.h"
//#include "../include/physics/Collision/Box.h"
//#include "../include/physics/Dynamics/Body.h"
//#include "../include/physics/Utils/Logger.h"
//
//void RunRealBodyTreeTest() {
//    Logger::Info("Starting Real Body Tree Integration Test...");
//    DynamicTree tree;
//    Box* shape = new Box(1.0f, 1.0f);
//
//    // 1. 创建 3 个真实的 Body
//    Body* bodyA = new Body(shape, 10.0f, 20.0f, 1.0f); // 坐标 (10, 20)
//    Body* bodyB = new Body(shape, -5.0f, 0.0f, 1.0f);  // 坐标 (-5, 0)
//    Body* bodyC = new Body(shape, 100.0f, 100.0f, 0.0f); // 静态 (100, 100)
//
//    // 2. 将 Body 注册到树中
//    Logger::Info("Action: Registering Bodies into DynamicTree...");
//    bodyA->setProxyId(tree.CreateProxy(bodyA->GetAABB(), bodyA));
//    bodyB->setProxyId(tree.CreateProxy(bodyB->GetAABB(), bodyB));
//    bodyC->setProxyId(tree.CreateProxy(bodyC->GetAABB(), bodyC));
//
//    // 3. 观察输出
//    tree.PrintPool();
//
//    // 4. 模拟 BodyB 移动并更新树
//    Logger::Info("Action: Moving BodyB and updating proxy...");
//    bodyB->SetPosition(Vector2(0.0f, 0.0f));
//    bodyB->updateAABB();
//
//    // 在 V2-Day 4 我们会写动态更新，现在我们先手动模拟删除再创建
//    tree.DestroyProxy(bodyB->getProxyId());
//    bodyB->setProxyId(tree.CreateProxy(bodyB->GetAABB(), bodyB));
//
//    tree.PrintPool();
//
//    Logger::Info("V2-Day 1 Integration Test Finished.");
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