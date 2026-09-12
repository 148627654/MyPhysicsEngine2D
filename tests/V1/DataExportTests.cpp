#include <iostream>
#include <vector>
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Collision/Circle.h"
#include "../include/physics/Utils/CSVExporter.h"
#include "../include/physics/Utils/Logger.h"

//int main() {
//    // 1. 初始化系统
//    Logger::Info("Starting Day 08 Test: Data Export & Logging");
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
//    bodyA->SetVelocity(Vector2(2.0f, 0.0f));
//
//    // 球 B：从右往左冲
//    Body* bodyB = new Body(circleShape, 5.0f, 0.0f, 1.0f);
//    bodyB->SetVelocity(Vector2(-2.0f, 0.0f));
//
//    world.AddBody(bodyA);
//    world.AddBody(bodyB);
//
//    Logger::Info("Bodies initialized. Body 0 at (-5,0), Body 1 at (5,0)");
//
//    // 3. 开始模拟 (120 帧，约 2 秒)
//    float dt = 1.0f / 60.0f;
//
//    for (int frame = 0; frame < 120; ++frame) {
//        // 物理步进
//        world.Step(dt);
//
//        // 记录当前帧所有物体状态到 008.csv
//        exporter.WriteFrame(frame, world.GetBodies());
//
//        // 逻辑提示：由于现在还没写“反弹”逻辑，球体会直接重叠穿过去
//        // 此时 World::Step 内部的 Collision::Dispatch 会持续触发
//        // 我们在 World::Step 后面手动调用 Logger 观察结果
//        // (注：如果在 World::Step 内部已经写了 Logger，这里就不需要了)
//    }
//
//    Logger::Info("Simulation finished. Data saved to output/008.csv");
//
//    // 清理
//    delete circleShape;
//    // 注意：实际项目中建议由 World 管理 Body 内存，这里仅作 Demo 
//
//    return 0;
//}