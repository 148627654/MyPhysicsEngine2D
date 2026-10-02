#include <iostream>
#include <vector>
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Collision/Circle.h"
#include "../include/physics/Utils/CSVExporter.h"
#include "../include/physics/Utils/Logger.h"

//int main() {
//    // 1. 初始化系统
//    Logger::info("Starting Day 08 Test: Data Export & Logging");
//
//    // 创建世界（无重力，方便观察匀速运动）
//    World world(Vector2(0, 0));
//
//    // 创建 CSV 记录器，命名为 008.csv
//    CSVExporter exporter("output/008.csv");
//
//    // 2. 创建两个对冲的球体
//    Circle* circleShape = new Circle(1.0f); // 半径 1.0
//
//    // 球 A：从左往右冲
//    Body* bodyA = new Body(circleShape, -5.0f, 0.0f, 1.0f);
//    bodyA->setVelocity(Vector2(2.0f, 0.0f));
//
//    // 球 B：从右往左冲
//    Body* bodyB = new Body(circleShape, 5.0f, 0.0f, 1.0f);
//    bodyB->setVelocity(Vector2(-2.0f, 0.0f));
//
//    world.addBody(bodyA);
//    world.addBody(bodyB);
//
//    Logger::info("Bodies initialized. Body 0 at (-5,0), Body 1 at (5,0)");
//
//    // 3. 开始模拟 (120 帧，约 2 秒)
//    float dt = 1.0f / 60.0f;
//
//    for (int frame = 0; frame < 120; ++frame) {
//        // 物理步进
//        world.step(dt);
//
//        // 记录当前帧所有物体状态到 008.csv
//        exporter.writeFrame(frame, world.getBodies());
//
//        // 逻辑提示：由于现在还没写“反弹”逻辑，球体会直接重叠穿过去
//        // 此时 World::step 内部的 Collision::dispatch 会持续触发
//        // 我们在 World::step 后面手动调用 Logger 观察结果
//        // (注：如果在 World::step 内部已经写了 Logger，这里就不需要了)
//    }
//
//    Logger::info("Simulation finished. Data saved to output/008.csv");
//
//    // 清理
//    delete circleShape;
//    // 注意：实际项目中建议由 World 管理 Body 内存，这里仅作 Demo 
//
//    return 0;
//}