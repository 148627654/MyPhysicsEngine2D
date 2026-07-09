#include <iostream>
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Collision/Box.h"

#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Utils/CSVExporter.h" // 包含你的工具类

int main() {
    // 1. 初始化物理世界
    World world(Vector2(0, -9.81f));

    // 2. 创建一个矩形刚体 (2x2, 密度 1.0, 位置 (0, 20))
    Box* boxShape = new Box(2.0f, 2.0f);
    Body* body = new Body(boxShape, 0.0f, 20.0f, 1.0f);
    body->SetVelocity(Vector2(5.0f, 0.0f));

    // 3. 施加一个偏心力，让它转起来
    // 在右上角 (1, 21) 施加向左的力，会产生顺时针转矩
    body->ApplyForceAtPoint(Vector2(-50.0f, 0.0f), Vector2(1.0f, 21.0f));

    world.AddBody(body);

    // 4. 使用你的 CSVExporter
    CSVExporter exporter("simulation_day03.csv");

    // 5. 模拟 2 秒 (60 FPS)
    float dt = 1.0f / 60.0f;
    for (int i = 0; i <= 120; ++i) {
        float currentTime = i * dt;

        // 调用你的工具类记录数据
        // 注意：如果 rotation 是私有的，请先在 Body.h 里加个 GetRotation()
        exporter.AddData(
            currentTime,
            body->GetPosition(),
            body->GetVelocity(),
            body->GetRotation()
        );

        // 物理步进
        world.Step(dt);
    }

    std::cout << "模拟成功！数据已记录至 simulation_day03.csv" << std::endl;

    // 清理
    delete body;
    delete boxShape;

    return 0;
}