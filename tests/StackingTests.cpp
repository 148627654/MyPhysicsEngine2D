#include <iostream>
#include <vector>
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Utils/CSVExporter.h"
#include "../include/physics/Utils/Logger.h"

int main() {
    Logger::Info("Starting Day 11 Test: Positional Correction (Stacking)");

    // 1. 初始化世界（标准重力）
    World world(Vector2(0, -9.81f));
    CSVExporter exporter("output/011.csv");

    // 2. 创建形状
    Box* boxShape = new Box(200.0f, 2.0f); // 2x2 的正方形

    // 3. 创建实体
    // 实体 0: 静态地面 (位于 y = -5)
    Body* ground = new Body(boxShape, 0.0f, -5.0f, 0.0f);
    ground->setRestitution(0.2f); // 低弹性，方便静止
    world.AddBody(ground);

    // 实体 1: 动态方块 (从空中掉落，初始 y = 5)
    Body* crate = new Body(boxShape, 0.1f, 5.0f, 1.0f); // 稍微偏移x轴，增加趣味性
    crate->setRestitution(0.3f);
    world.AddBody(crate);

    Logger::Info("Scene setup: Dynamic box falling onto static ground.");

    // 4. 模拟循环 (300 帧，约 5 秒)
    // 足够长的时间来观察它是否稳定在地表
    float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < 300; ++frame) {
        world.Step(dt);
        exporter.WriteFrame(frame, world.GetBodies());

        if (frame % 60 == 0) {
            std::string msg = "Frame " + std::to_string(frame) +
                " | Box Y: " + std::to_string(crate->GetPosition().getY());
            Logger::Info(msg);
        }
    }

    Logger::Info("Simulation finished. Data saved to output/011.csv");

    delete boxShape;
    return 0;
}