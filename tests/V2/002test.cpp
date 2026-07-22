#include "../include/physics/Collision/DynamicTree.h"
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Utils/Logger.h"

void RunDay2InsertTest() {
    Logger::Info("Starting V2-Day 2: InsertLeaf Logic Test");
    DynamicTree tree;
    Box* shape = new Box(1.0f, 1.0f); // 1x1 的方块

    // 1. 插入第一个物体 (10, 10)
    Body* b1 = new Body(shape, 10.0f, 10.0f, 1.0f);
    int32_t p1 = tree.CreateProxy(b1->GetAABB(), b1);
    Logger::Info("Inserted Body 1 at (10,10). Root should be 0.");
    tree.PrintPool(); // 检查 root 是否指向 0

    // 2. 插入第二个物体 (20, 20)
    Body* b2 = new Body(shape, 20.0f, 20.0f, 1.0f);
    int32_t p2 = tree.CreateProxy(b2->GetAABB(), b2);
    Logger::Info("Inserted Body 2 at (20,20). Root should now be an Internal Node.");
    tree.PrintPool();

    // 3. 验证 AABB 是否正确包裹
    int32_t rootId = tree.GetRoot(); // 需要在类里加个 GetRoot()
    AABB rootAABB = tree.GetNodeAABB(rootId); // 需要在类里加个 GetNodeAABB()

    Logger::Info("Root AABB Min: (" + std::to_string(rootAABB.min.getX()) + "," + std::to_string(rootAABB.min.getY()) + ")");
    Logger::Info("Root AABB Max: (" + std::to_string(rootAABB.max.getX()) + "," + std::to_string(rootAABB.max.getY()) + ")");

    // 预期：Body1 是 (9.5~10.5)，Body2 是 (19.5~20.5)
    // Root AABB 应该至少是 (9.5, 9.5) 到 (20.5, 20.5)
    if (rootAABB.min.getX() <= 9.51f && rootAABB.max.getX() >= 20.49f) {
        Logger::Info("SUCCESS: Root AABB correctly encapsulates both children!");
    }
    else {
        Logger::Error("FAILURE: Root AABB is too small!");
    }

    // 4. 插入第三个物体 (0, 0)
    Body* b3 = new Body(shape, 0.0f, 0.0f, 1.0f);
    tree.CreateProxy(b3->GetAABB(), b3);
    Logger::Info("Inserted Body 3 at (0,0). Verifying hierarchy...");
    tree.PrintPool();
    tree.Validate();

    Logger::Info("V2-Day 2 Test Completed.");
}

int main()
{
    RunDay2InsertTest();
    return 0;
}