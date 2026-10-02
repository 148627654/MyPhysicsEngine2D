//#include "../include/physics/Collision/DynamicTree.h"
//#include "../include/physics/Dynamics/Body.h"
//#include "../include/physics/Collision/Box.h"
//#include "../include/physics/Utils/Logger.h"
//
//void RunDay2InsertTest() {
//    Logger::info("Starting V2-Day 2: insertLeaf Logic Test");
//    DynamicTree tree;
//    Box* shape = new Box(1.0f, 1.0f); // 1x1 的方块
//
//    // 1. 插入第一个物体 (10, 10)
//    Body* b1 = new Body(shape, 10.0f, 10.0f, 1.0f);
//    int32_t p1 = tree.createProxy(b1->getAABB(), b1);
//    Logger::info("Inserted Body 1 at (10,10). Root should be 0.");
//    tree.printPool(); // 检查 root 是否指向 0
//
//    // 2. 插入第二个物体 (20, 20)
//    Body* b2 = new Body(shape, 20.0f, 20.0f, 1.0f);
//    int32_t p2 = tree.createProxy(b2->getAABB(), b2);
//    Logger::info("Inserted Body 2 at (20,20). Root should now be an Internal Node.");
//    tree.printPool();
//
//    // 3. 验证 AABB 是否正确包裹
//    int32_t rootId = tree.getRoot(); // 需要在类里加个 getRoot()
//    AABB rootAABB = tree.getNodeAABB(rootId); // 需要在类里加个 getNodeAABB()
//
//    Logger::info("Root AABB Min: (" + std::to_string(rootAABB.min.getX()) + "," + std::to_string(rootAABB.min.getY()) + ")");
//    Logger::info("Root AABB Max: (" + std::to_string(rootAABB.max.getX()) + "," + std::to_string(rootAABB.max.getY()) + ")");
//
//    // 预期：Body1 是 (9.5~10.5)，Body2 是 (19.5~20.5)
//    // Root AABB 应该至少是 (9.5, 9.5) 到 (20.5, 20.5)
//    if (rootAABB.min.getX() <= 9.51f && rootAABB.max.getX() >= 20.49f) {
//        Logger::info("SUCCESS: Root AABB correctly encapsulates both children!");
//    }
//    else {
//        Logger::error("FAILURE: Root AABB is too small!");
//    }
//
//    // 4. 插入第三个物体 (0, 0)
//    Body* b3 = new Body(shape, 0.0f, 0.0f, 1.0f);
//    tree.createProxy(b3->getAABB(), b3);
//    Logger::info("Inserted Body 3 at (0,0). Verifying hierarchy...");
//    tree.printPool();
//
//    Logger::info("V2-Day 2 Test Completed.");
//}
//
//int main()
//{
//    RunDay2InsertTest();
//    return 0;
//}