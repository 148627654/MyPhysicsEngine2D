#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"
#include "../Utils/CSVExporter.h"

void RunCollisionWakeupTest() {
    Logger::Info("=== Starting Day 08: Freefall Sleep & Collision Wakeup ===");
    CSVExporter exporter("output/v2_008_final.csv");

    // 1. 环境初始化
    Vector2 gravity(0.0f, -9.8f);
    World world(gravity);
    float dt = 1.0f / 60.0f;

    // 2. 创建地面 (ID: 0)
    Box* groundShape = new Box(40.0f, 2.0f);
    Body* ground = new Body(groundShape, 0.0f, -1.0f, 0.0f); // 静态物体
    world.AddBody(ground);

    // 3. 创建被动箱子 Box A (ID: 1) - 从高处自由落体
    Box* boxShape = new Box(1.0f, 1.0f);
    Body* boxA = new Body(boxShape, 0.0f, 6.0f, 1.0f);
    boxA->setRestitution(0.3f); // 给一点点弹力，看它如何自然静止
    boxA->setFriction(0.5f);
    world.AddBody(boxA);

    Logger::Info("Phase 1: Box A is free falling from y=6.0...");

    // --- 阶段 1：等待 Box A 落地、反弹、最终入睡 ---
    int frame = 0;
    bool boxASlept = false;
    // 模拟上限设为 600 帧 (10秒)，通常 2-3 秒就睡着了
    for (; frame < 600; ++frame) {
        world.Step(dt);
        exporter.WriteFrame(frame, world.GetBodies());

        if (!boxASlept && !boxA->IsAwake()) {
            Logger::Info("SUCCESS: Box A settled and fell asleep at frame " + std::to_string(frame));
            boxASlept = true;
            // 额外多跑 30 帧，确信它睡得很死
            for (int j = 0; j < 30; ++j) {
                frame++;
                world.Step(dt);
                exporter.WriteFrame(frame, world.GetBodies());
            }
            break;
        }
    }

    if (!boxASlept) {
        Logger::Error("FAILURE: Box A is still jittering and won't sleep. Check thresholds!");
        return;
    }

    // --- 阶段 2：在左上方投放 Box B，瞄准睡着的 Box A ---
    Logger::Info("Phase 2: Spawning Box B at (-5, 8) to strike Box A...");

    // 创建入侵者 Box B (ID: 2)
    Body* boxB = new Body(boxShape, -5.0f, 8.0f, 1.2f); // 稍微重一点，撞击感更强
    // 设置初速度：向右下方射出
    boxB->SetVelocity(Vector2(6.0f, -4.0f));
    boxB->setAngularVelocity(2.0f); // 旋转着砸下去更真实
    world.AddBody(boxB);

    bool wokeUp = false;
    // 继续模拟 180 帧 (3秒) 观察碰撞后的混乱现场
    for (int i = 0; i < 180; ++i) {
        frame++;
        world.Step(dt);
        exporter.WriteFrame(frame, world.GetBodies());

        if (!wokeUp && boxA->IsAwake()) {
            Logger::Info("BINGO! Box A was AWAKENED by the strike at frame " + std::to_string(frame));
            wokeUp = true;
        }
    }

    if (wokeUp) {
        Logger::Info("Simulation finished perfectly. File saved: output/v2_008_final.csv");
    }
    else {
        Logger::Error("FAILURE: Impact occurred but Box A remained in deep sleep!");
    }
}

//int main() {
//    // 确保目录存在
//    // _mkdir("output"); 
//    RunCollisionWakeupTest();
//    return 0;
//}