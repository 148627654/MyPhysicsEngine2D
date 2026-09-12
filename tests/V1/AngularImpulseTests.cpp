#include <iostream>
#include <vector>
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Utils/CSVExporter.h"
#include "../include/physics/Utils/Logger.h"

//int main() {
//    Logger::Info("Starting Day 10 Test: Angular Impulse & Rotation");
//
//    // 1. 初始化世界（标准重力）
//    World world(Vector2(0, -9.81f));
//    CSVExporter exporter("output/010.csv");
//
//    // 2. 创建形状
//    // 地面：宽 20，高 1
//    Box* groundShape = new Box(20.0f, 1.0f);
//    // 木板：长 4，宽 0.5 (长条状更容易观察旋转)
//    Box* plankShape = new Box(4.0f, 0.5f);
//
//    // 3. 创建实体
//    // 地面 (静态)
//    Body* ground = new Body(groundShape, 0.0f, -5.0f, 0.0f);
//    ground->setRestitution(0.6f);
//    world.AddBody(ground);
//
//    // 动态木板
//    // 初始位置 (0, 0)，初始旋转 30 度 (约 0.523 弧度)
//    Body* plank = new Body(plankShape, 0.0f, 0.0f, 1.0f);
//    plank->SetRotation(0.523f);
//    plank->setRestitution(0.7f); // 较好的弹性
//    world.AddBody(plank);
//
//    Logger::Info("Scene setup: Tilted plank at (0,0), Ground at -5.");
//
//    // 4. 模拟循环 (200 帧，约 3.3 秒)
//    float dt = 1.0f / 60.0f;
//    for (int frame = 0; frame < 200; ++frame) {
//        world.Step(dt);
//        exporter.WriteFrame(frame, world.GetBodies());
//
//        if (frame % 40 == 0) {
//            std::string msg = "Frame " + std::to_string(frame) +
//                " | AngVel: " + std::to_string(plank->getAngularVelocity());
//            Logger::Info(msg);
//        }
//    }
//
//    Logger::Info("Simulation finished. Data saved to output/010.csv");
//
//    delete groundShape;
//    delete plankShape;
//
//    return 0;
//}