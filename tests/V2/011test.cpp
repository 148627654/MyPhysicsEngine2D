#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"
#include "../Utils/CSVExporter.h"
#include <iostream>

void TestBulletTunneling() {
    Logger::info("Starting Day 11 Test: High-Speed Tunneling Detection...");
    CSVExporter exporter("output/v2_011.csv");

    // 1. 初始化世界（关闭重力，便于观察纯水平运动）
    World world(Vector2(0, 0));
    float dt = 1.0f / 60.0f;

    // 2. 创建极薄的墙 (Static, ID: 0)
    Body* wall = new Body(new Box(0.1f, 10.0f), 0.0f, 0.0f, 0.0f);
    world.addBody(wall);
    Logger::info("Static wall created at X=0, Thickness=0.1");

    // 3. 创建高速子弹 (Dynamic, ID: 1)
    // 初始位置在 -5.0，速度 120，每帧移动 2.0
    Body* bullet = new Body(new Box(0.5f, 0.5f), -5.0f, 0.0f, 1.0f);
    bullet->setBullet(true); // 【核心】：开启扫掠包围盒
    bullet->setVelocity(Vector2(120.0f, 0.0f));
    world.addBody(bullet);
    Logger::info("Bullet spawned at X=-5.0, Vel=120.0 (2.0 per frame)");

    bool detectedPotentialCollision = false;

    // 4. 模拟 10 帧
    for (int frame = 0; frame < 10; ++frame) {
        // 在 step 执行前记录一次
        exporter.writeFrame(frame, world.getBodies());

        // 执行物理步进
        world.step(dt);

        // 检查宽相是否捕捉到了碰撞对
        auto const& contactMap = world.getContactMap();
        if (!detectedPotentialCollision && !contactMap.empty()) {
            Logger::info("BINGO! BroadPhase caught the bullet at frame " + std::to_string(frame));
            Logger::info("Bullet position: " + std::to_string(bullet->getPosition().getX()));
            detectedPotentialCollision = true;
        }
    }

    if (detectedPotentialCollision) {
        Logger::info("SUCCESS: Swept AABB successfully captured the high-speed trajectory!");
    }
    else {
        Logger::error("FAILURE: Bullet tunneled through the wall without broadphase detection!");
    }

    Logger::info("Check 'output/v2_011.csv' for trajectory analysis.");
}

//int main() {
//    TestBulletTunneling();
//    return 0;
//}