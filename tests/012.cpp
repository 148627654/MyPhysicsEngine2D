#include <iostream>
#include <vector>
#include <string>
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Collision/Circle.h"
#include "../include/physics/Utils/CSVExporter.h"
#include "../include/physics/Utils/Logger.h"

int main() {
    Logger::Info("Starting Day 12 Test: Sequential Impulses & Friction");

    // 1. 初始化世界（标准重力）
    World world(Vector2(0, -9.81f));
    // 修改输出文件名为 012.csv
    CSVExporter exporter("output/012.csv");

    // 2. 创建共享形状
    Box* groundShape = new Box(40.0f, 2.0f);   // 宽大的地面
    Box* crateShape = new Box(2.0f, 2.0f);    // 标准方块
    Box* plankShape = new Box(6.0f, 0.5f);    // 长木板（测试多点接触）

    // 3. 场景搭建

    // --- 实体 0: 静态地面 (y = -10) ---
    Body* ground = new Body(groundShape, 0.0f, -10.0f, 0.0f);
    ground->setFriction(0.6f);      // 地面设置较大摩擦
    ground->setRestitution(0.1f);  // 低弹性
    world.AddBody(ground);

    // --- 实体 1: 滑行方块 (测试摩擦力制动) ---
    // 位于地面上，给一个 15.0 的水平初速度
    Body* slider = new Body(crateShape, -15.0f, -8.0f, 1.0f);
    slider->setFriction(0.3f);     // 方块摩擦力
    slider->SetVelocity(Vector2(15.0f, 0.0f));
    world.AddBody(slider);

    // --- 实体 2: 倾斜长木板 (测试多点碰撞稳定性) ---
    // 45度角掉落。有了多点碰撞，它应该“拍”在地上而不是一直旋转。
    Body* plank = new Body(plankShape, 5.0f, 5.0f, 0.5f);
    plank->SetRotation(0.785f);    // 45 deg
    plank->setFriction(0.4f);
    world.AddBody(plank);

    // --- 实体 3 & 4: 简单堆叠 (测试迭代解算) ---
    // 两个方块叠在一起
    Body* stackBottom = new Body(crateShape, 0.0f, 0.0f, 1.0f);
    Body* stackTop = new Body(crateShape, 0.05f, 4.0f, 1.0f); // 稍微偏移增加挑战
    world.AddBody(stackBottom);
    world.AddBody(stackTop);

    Logger::Info("Scene setup: 1 Slider, 1 Tilted Plank, 2 Stacked Boxes.");

    // 4. 模拟循环 (420 帧，约 7 秒)
    float dt = 1.0f / 60.0f;
    for (int frame = 0; frame < 420; ++frame) {
        world.Step(dt);

        // 记录所有数据到 CSV
        exporter.WriteFrame(frame, world.GetBodies());

        // 每秒打印一次关键数据进行监控
        if (frame % 60 == 0) {
            std::string msg = "Frame " + std::to_string(frame) +
                " | Slider VelX: " + std::to_string(slider->GetVelocity().getX()) +
                " | Plank Angle: " + std::to_string(plank->GetRotation());
            Logger::Info(msg);
        }
    }

    Logger::Info("Simulation finished. Data saved to output/012.csv");

    // 5. 清理内存
    delete groundShape;
    delete crateShape;
    delete plankShape;

    return 0;
}