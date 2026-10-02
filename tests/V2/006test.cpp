//#include "../include/physics/Dynamics/World.h"
//#include "../include/physics/Collision/Box.h"
//#include "../include/physics/Collision/Circle.h"
//#include "../include/physics/Utils/Logger.h"
//#include "../include/physics/Utils/CSVExporter.h" // 确保包含导出器
//#include <chrono>
//
//void RunDay6FinalIntegrationTest() {
//    Logger::info(">>> [V2-Day 6] Full Automation & Visualization Test <<<");
//
//    // 1. 初始化世界与导出器
//    World world(Vector2(0, -9.8f));
//    CSVExporter exporter("output/v2_day6_sim.csv");
//
//    Box* boxShape = new Box(1.0f, 1.0f);
//    Circle* bulletShape = new Circle(0.5f);
//
//    // 2. 自动化构建墙体 (100个方块)
//    Logger::info("step 1: Building a 'Wall' of 100 boxes...");
//    for (int y = 0; y < 10; ++y) {
//        for (int x = 0; x < 10; ++x) {
//            // 坐标 (x*1.1, y*1.1)，预留微小缝隙防止重叠爆炸
//            Body* b = new Body(boxShape, x * 1.1f, y * 1.1f);
//            world.addBody(b);
//        }
//    }
//
//    // 3. 执行单次射线检测 (功能性测试)
//    Logger::info("step 2: Running rayCast across the wall center...");
//    world.rayCast(Vector2(-5.0f, 5.5f), Vector2(15.0f, 5.5f));
//
//    // 4. 模拟主循环 (可视化核心)
//    Logger::info("step 3: Starting Simulation Loop (200 frames)...");
//    const float dt = 0.016f;
//    const int totalFrames = 200;
//
//    for (int frame = 0; frame < totalFrames; ++frame) {
//
//        // 在第 50 帧时发射子弹，增加戏剧性效果
//        if (frame == 50) {
//            Logger::info("Frame 50: LAUNCHING HIGH-SPEED BULLET!");
//            Body* bullet = new Body(bulletShape, -15.0f, 5.5f); // 从左侧远处射入
//            bullet->setVelocity(Vector2(60.0f, 0.0f)); // 速度 60
//            world.addBody(bullet);
//        }
//
//        // 性能计时
//        auto start = std::chrono::high_resolution_clock::now();
//
//        world.step(dt); // 这一步包含了位置积分、宽相同步、窄相回调和冲量解算
//
//        auto end = std::chrono::high_resolution_clock::now();
//        std::chrono::duration<double, std::milli> elapsed = end - start;
//
//        // 每一帧将所有物体状态写入 CSV
//        exporter.writeFrame(frame, world.getBodies());
//
//        // 仅在关键帧打印耗时
//        if (frame % 50 == 0) {
//            Logger::info("Frame " + std::to_string(frame) + " step Time: " + std::to_string(elapsed.count()) + "ms");
//        }
//    }
//
//    Logger::info("Simulation finished. Data saved to 'v2_day6_sim.csv'.");
//
//    // --- 5. 使用你实现的 removeBody 进行安全清理 ---
//    Logger::info("step 5: Cleaning up world...");
//
//    // 注意：不能直接用 for(auto b : world.getBodies()) { world.removeBody(b); }
//    // 因为 removeBody 内部会修改 vector 导致迭代器失效。
//    // 正确做法是从后往前删，或者循环删除末尾元素：
//    while (!world.getBodies().empty()) {
//        world.removeBody(world.getBodies().back());
//    }
//
//    delete boxShape;
//    delete bulletShape;
//
//    Logger::info(">>> V2-Day 6 Test Completed Successfully. <<<");
//}
//
////int main()
////{
////    RunDay6FinalIntegrationTest();
////    return 0;
////}