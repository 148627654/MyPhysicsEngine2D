#include <iostream>
#include <vector>
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Collision/Circle.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Utils/CSVExporter.h"
#include "../include/physics/Utils/Logger.h"
#include "../include/physics/Dynamics/Solver.h"

//int main() {
//    Logger::Info("Starting Day 09 Test: Multi-Body Impulse Resolution");
//
//    // 1. 初始化物理世界 (给一点重力，观察下落反弹)
//    World world(Vector2(0, -9.81f));
//    CSVExporter exporter("output/009_multi.csv");
//
//    // 2. 创建形状
//    Circle* circleShape = new Circle(1.0f);
//    Box* groundShape = new Box(20.0f, 1.0f); // 一个宽 20, 高 1 的地面
//
//    // 3. 创建实体
//
//    // --- 实体 0: 静态地面 ---
//    // 密度设为 0，Body 构造函数会将其判定为静态 (invMass = 0)
//    Body* ground = new Body(groundShape, 0.0f, -5.0f, 0.0f);
//    ground->setRestitution(0.8f); // 比较有弹性
//    world.AddBody(ground);
//
//    // --- 实体 1: 动态球 A (从左边飞过来) ---
//    Body* ballA = new Body(circleShape, -6.0f, 0.0f, 1.0f);
//    ballA->SetVelocity(Vector2(5.0f, 0.0f));
//    ballA->setRestitution(0.8f);
//    world.AddBody(ballA);
//
//    // --- 实体 2: 动态球 B (从右边迎面撞击) ---
//    Body* ballB = new Body(circleShape, 6.0f, 0.0f, 1.0f);
//    ballB->SetVelocity(Vector2(-5.0f, 0.0f));
//    ballB->setRestitution(0.8f);
//    world.AddBody(ballB);
//
//    // --- 实体 3: 动态球 C (悬浮在中间，等待被撞) ---
//    Body* ballC = new Body(circleShape, 0.0f, 2.0f, 1.0f);
//    ballC->SetVelocity(Vector2(0.0f, 0.0f));
//    ballC->setRestitution(0.8f);
//    world.AddBody(ballC);
//
//    Logger::Info("Scene setup complete. 3 dynamic balls, 1 static ground.");
//
//    // 4. 模拟循环 (3 秒，180 帧)
//    float dt = 1.0f / 60.0f;
//    for (int frame = 0; frame < 180; ++frame) {
//        world.Step(dt);
//
//        // 记录所有物体状态
//        exporter.WriteFrame(frame, world.GetBodies());
//
//        // 每隔 30 帧打印一下球 A 的位置，方便在控制台观察
//        if (frame % 30 == 0) {
//            std::string msg = "Frame " + std::to_string(frame) + " | Ball A Y: " + std::to_string(ballA->GetPosition().getY());
//            Logger::Info(msg);
//        }
//    }
//
//    Logger::Info("Simulation finished. Data saved to output/009_multi.csv");
//
//    // 清理 (实际代码建议由 World 析构函数自动清理)
//    delete circleShape;
//    delete groundShape;
//
//    return 0;
//}
