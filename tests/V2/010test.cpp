#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"
#include "../Utils/CSVExporter.h"

// --- 测试 1: 链式唤醒 (多米诺效应) ---
void TestChainWakeup() {
    Logger::info("Starting Day 10 Test 1: Chain Wakeup Reaction...");
    CSVExporter exporter("output/v2_010_chain.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 1. 地面
    world.addBody(new Body(new Box(50, 2), 0, -1, 0));

    // 2. 创建 5 个间距很小的睡眠方块
    std::vector<Body*> chain;
    for (int i = 0; i < 5; ++i) {
        Body* b = new Body(new Box(1, 1), (float)i * 1.1f, 0.5f, 1.0f);
        b->setRestitution(0.1f);
        world.addBody(b);
        chain.push_back(b);
    }

    // 3. 等待全部入睡
    int frame = 0;
    while (frame < 300) {
        world.step(dt);
        if (frame % 2 == 0) exporter.writeFrame(frame, world.getBodies());
        frame++;

        bool allSleep = true;
        for (auto b : chain) if (b->isAwake()) allSleep = false;
        if (allSleep) break;
    }
    Logger::info("All blocks in chain are now sleeping.");

    // 4. 发射一个“子弹”撞击第一个方块
    Body* bullet = new Body(new Box(0.5f, 0.5f), -5.0f, 0.5f, 2.0f);
    bullet->setVelocity(Vector2(15.0f, 0.0f));
    world.addBody(bullet);
    Logger::info("Bullet fired at Block 0!");

    // 5. 观察连锁反应
    for (int i = 0; i < 120; ++i) {
        world.step(dt);
        exporter.writeFrame(frame++, world.getBodies());
    }
    Logger::info("Chain reaction complete. Check 'v2_010_chain.csv'.");
}

// --- 测试 2: 15 层塔稳定性与底座拆除 ---
void TestTowerStability() {
    Logger::info("Starting Day 10 Test 2: 15-Layer Tower & Base Removal...");
    CSVExporter exporter("output/v2_010_tower.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 1. 地面
    world.addBody(new Body(new Box(20, 2), 0, -1, 0));

    // 2. 精准堆叠 15 个方块
    std::vector<Body*> tower;
    for (int i = 0; i < 15; ++i) {
        // 坐标：X=0, Y=0.5, 1.5, 2.5...
        Body* b = new Body(new Box(1, 1), 0.0f, 0.5f + (float)i * 1.0f, 1.0f);
        b->setFriction(0.6f);
        b->setRestitution(0.0f); // 稳定堆叠建议 0 弹力
        world.addBody(b);
        tower.push_back(b);
    }

    // 3. 稳定期：观察 Warm Starting 带来的快速入睡
    int frame = 0;
    Logger::info("Phase 1: Stabilizing 15-layer tower...");
    for (; frame < 400; ++frame) {
        world.step(dt);
        if (frame % 5 == 0) exporter.writeFrame(frame, world.getBodies());

        bool allSlept = true;
        for (auto b : tower) if (b->isAwake()) allSlept = false;
        if (allSlept) {
            Logger::info("Tower fully asleep at frame " + std::to_string(frame));
            break;
        }
    }

    // 4. 拆除地基：removeBody(tower[0])
    Logger::info("Phase 2: SUDDEN IMPACT! Removing the base block...");
    Body* base = tower[0];
    world.removeBody(base);

    // 5. 观察整座塔坠落
    for (int i = 0; i < 150; ++i) {
        world.step(dt);
        exporter.writeFrame(frame++, world.getBodies());
    }
    Logger::info("Tower collapse finished. Check 'v2_010_tower.csv'.");
}

//int main() {
//    try {
//        TestChainWakeup();
//        TestTowerStability();
//    }
//    catch (...) {
//        Logger::error("An error occurred during Day 10 tests.");
//    }
//    return 0;
//}