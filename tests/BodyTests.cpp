#include <iostream>
#include <iomanip> // 用于格式化输出
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Dynamics/World.h"
#include "../include/physics/Common/Setting.h"
#include "../include/physics/Utils/CSVExporter.h"

void RunExportSimulation() {
    World world;
    Body* ball = new Body(Vector2(0, 10), 1.0f);
    ball->SetVelocity(Vector2(2.0f, 0.0f));
    world.AddBody(ball);

    // 创建导出器，文件会生成在项目运行目录下
    CSVExporter exporter("simulation_data.csv");

    float totalTime = 0.0f;
    float duration = 2.0f; // 模拟2秒
    int frames = static_cast<int>(duration / Settings::DT);

    for (int i = 0; i < frames; ++i) {
        // 记录当前状态
        exporter.AddData(totalTime, ball->GetPosition(), ball->GetVelocity(), 0.0f);

        // 物理步进
        world.Step(Settings::DT);
        totalTime += Settings::DT;
    }

    delete ball;
    std::cout << "Data exported to simulation_data.csv\n";
}

int main() {
    RunExportSimulation();
    return 0;
}