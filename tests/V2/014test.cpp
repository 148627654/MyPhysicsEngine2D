#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Box.h"
#include "../Collision/Circle.h"
#include "../Utils/Logger.h"
#include "../Utils/CSVExporter.h"
#include <ctime>

// --- 场景 1: 1000 个球体乱跳 (测试 AABB 树和窄相性能) ---
void Run1000BallsStressTest() {
    Logger::Info(">>> Starting 1000-Ball Chaos Test...");
    CSVExporter exporter("output/v2_014_balls_1000.csv");
    World world(Vector2(0, 0)); // 关闭重力，防止全部堆在底部
    float dt = 1.0f / 60.0f;
    
    srand((unsigned int)time(0));

    // 1. 创建封闭容器 (4 堵墙)
    float size = 40.0f;
    float wallThickness = 5.0f; // 从 1.0 增加到 5.0
    world.AddBody(new Body(new Box(size * 2, wallThickness), 0, size + wallThickness / 2, 0)); // 顶
    world.AddBody(new Body(new Box(size * 2, wallThickness), 0, -size- wallThickness / 2, 0));   // 底
    world.AddBody(new Body(new Box(wallThickness, size * 2), -size- wallThickness / 2, 0, 0));   // 左
    world.AddBody(new Body(new Box(wallThickness, size * 2), size+ wallThickness / 2, 0, 0));    // 右

    // 2. 随机生成 1000 个圆球
    for (int i = 0; i < 1000; ++i) {
        float x = (rand() % (int)(size * 180) / 100.0f) - size * 0.9f;
        float y = (rand() % (int)(size * 180) / 100.0f) - size * 0.9f;
        Body* b = new Body(new Circle(0.4f), x, y, 1.0f);
        
        // 给一个随机初速度，让它们乱撞
        float vx = (rand() % 200 / 10.0f) - 15.0f;
        float vy = (rand() % 200 / 10.0f) - 15.0f;
        b->SetVelocity(Vector2(vx, vy));
        b->setRestitution(0.2f); // 高弹性
        b->SetBullet(true);
        world.AddBody(b);
    }

    // 3. 运行 600 帧 (10秒)
    for (int frame = 0; frame < 600; ++frame) {
        world.Step(dt);
        if (frame % 100 == 0) 
            exporter.WriteFrame(frame, world.GetBodies());

        // 每隔 1 秒打印一次性能报告
        if (frame % 60 == 0) {
            Logger::Info("--- Frame " + std::to_string(frame) + " Performance ---");
            world.GetProfiler().PrintReport();
        }
    }
}

// --- 场景 2: 2000 个盒子入睡 (测试睡眠机制和零开销拦截) ---
//void Run2000BoxesSleepTest() {
//    Logger::Info(">>> Starting 2000-Box Sleep Test...");
//    CSVExporter exporter("output/v2_014_boxes_2000.csv");
//    World world(Vector2(0, -9.8f)); // 开启重力
//    float dt = 1.0f / 60.0f;
//    srand(42);
//
//    // 1. 创建大型地面
//    world.AddBody(new Body(new Box(200, 2), 0, -2, 0));
//
//    // 2. 网格化生成 2000 个盒子 (40列 x 50行)
//    for (int y = 0; y < 50; ++y) {
//        for (int x = 0; x < 40; ++x) {
//            float px = (x - 20) * 1.2f;
//            float py = y * 1.2f + 5.0f;
//            Body* b = new Body(new Box(0.5f, 0.5f), px, py, 1.0f);
//            b->setRestitution(0.0f); // 低弹性加速入睡
//            b->setFriction(0.5f);
//            b->setSleepAllow(true);
//            world.AddBody(b);
//        }
//    }
//
//    // 3. 模拟 1200 帧 (20秒)，观察耗时如何从高峰掉到地平线
//    for (int frame = 0; frame < 1200; ++frame) {
//        world.Step(dt);
//
//        // 每隔 5 秒保存一次数据（2000个物体数据量大，不建议每帧写）
//        if (frame % 300 == 0) exporter.WriteFrame(frame, world.GetBodies());
//
//        // 每隔 2 秒打印性能快照
//        if (frame % 120 == 0) {
//            Logger::Info("--- Frame " + std::to_string(frame) + " Status ---");
//            world.GetProfiler().PrintReport();
//        }
//    }
//}

int main() {
    // 运行压力测试
    Run1000BallsStressTest();
    //Run2000BoxesSleepTest();
    return 0;
}