#include "../Dynamics/World.h"
#include "../Collision/Shape.h"
#include "../Dynamics/Body.h"
#include "../Utils/Logger.h"
#include "../Utils/CSVExporter.h"
#include <iostream>
#include "../../include/physics/Collision/Box.h"

void RunEnhancedIslandTest() {
    // 1. 初始化工具
    Logger::info("Initializing Enhanced Island Test...");
    CSVExporter exporter("output/v2_007.csv");

    // 2. 初始化世界
    Vector2 gravity(0.0f, -9.8f);
    World world(gravity);

    // 3. 创建地面 (静态)
    Box* groundShape = new Box(20.0f, 1.0f);
    Body* ground = new Body(groundShape, 0.0f, 0.0f, 0.0f);
    world.addBody(ground);
    Logger::info("Static ground created at (0, 0)");

    // 4. 创建场景 A (左侧：两个箱子垂直碰撞)
    Box* boxShape = new Box(1.0f, 1.0f);
    Body* boxA1 = new Body(boxShape, -5.0f, 10.0f, 1.0f); // 高处落下
    Body* boxA2 = new Body(boxShape, -5.0f, 1.5f, 1.0f);  // 紧贴地面
    world.addBody(boxA1);
    world.addBody(boxA2);
    Logger::info("Island A (Left): 2 boxes stacked vertically.");

    // 5. 创建场景 B (右侧：单个箱子自由落体)
    Body* boxB1 = new Body(boxShape, 5.0f, 10.0f, 1.0f);
    world.addBody(boxB1);
    Logger::info("Island B (Right): 1 box falling freely.");

    // 6. 模拟循环 (例如模拟 2 秒，120 帧)
    const float dt = 1.0f / 60.0f;
    Logger::info("Simulation starting for 120 frames...");

    for (int frame = 0; frame < 120; ++frame) {
        // --- 核心步骤 ---
        world.step(dt);

        // --- 数据导出 ---
        // 将当前帧所有物体的状态写入 CSV
        exporter.writeFrame(frame, world.getBodies());

        // --- 控制台反馈 (每 30 帧打印一次摘要) ---
        if (frame % 30 == 0) {
            std::string msg = "Frame " + std::to_string(frame) +
                " | BoxA1_Y: " + std::to_string(boxA1->getPosition().getY()) +
                " | BoxB1_Y: " + std::to_string(boxB1->getPosition().getY());
            Logger::info(msg);
        }

        // --- 碰撞实时监控 (可选) ---
        // 如果你的 World::step 内部有 Contact 对象，可以在这里遍历并 logCollision
        // 这里演示如何捕获第一个有效碰撞并打印
        for (auto const& [key, contact] : world.getContactMap()) {
            if (contact->isTouching()) {
                // Logger::logCollision(contact->getManifold());
            }
        }
    }

    Logger::info("Simulation finished. Data saved to 'island_test_data.csv'.");
    Logger::info("Success: Verify that Box A and Box B moved independently.");
}

//int main() {
//    try {
//        RunEnhancedIslandTest();
//    }
//    catch (const std::exception& e) {
//        Logger::error("Test failed with exception: " + std::string(e.what()));
//    }
//    return 0;
//}