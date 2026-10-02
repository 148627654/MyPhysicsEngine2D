#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"
#include "../Utils/CSVExporter.h"

// --- 测试 1: 悬空唤醒测试 ---
void TestSuspensionWakeup() {
    Logger::info("Starting Test 1: Suspension Wakeup...");
    CSVExporter exporter("output/v2_009_001.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* ground = new Body(new Box(20, 2), 0, -1, 0); world.addBody(ground);
    Body* box1 = new Body(new Box(1, 1), 0, 0.5f, 1.0f); world.addBody(box1);
    Body* box2 = new Body(new Box(1, 1), 0, 1.5f, 1.0f); world.addBody(box2);

    int frame = 0;
    // 1. 等待入睡
    while (frame < 500) {
        world.step(dt);
        exporter.writeFrame(frame++, world.getBodies());
        if (!box1->isAwake() && !box2->isAwake()) break;
    }
    Logger::info("Boxes are sleeping. Now removing box1 (the base)...");

    // 2. 移除底座
    world.removeBody(box1);

    // 3. 观察 box2 是否被唤醒
    for (int i = 0; i < 60; ++i) {
        world.step(dt);
        exporter.writeFrame(frame++, world.getBodies());
    }

    if (box2->isAwake() && box2->getPosition().getY() < 1.4f) {
        Logger::info("SUCCESS: Box2 was awakened and fell down!");
    }
    else {
        Logger::error("FAILURE: Box2 is still floating!");
    }
}

// --- 测试 2: 类型转换测试 ---
void TestTypeConversion() {
    Logger::info("Starting Test 2: Type Conversion Wakeup...");
    CSVExporter exporter("output/v2_009_002.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 创建一个静态平台
    Body* platform = new Body(new Box(5, 1), 0, 0, 0); world.addBody(platform);
    Body* box = new Body(new Box(1, 1), 0, 1.0f, 1.0f); world.addBody(box);

    int frame = 0;
    while (frame < 300) {
        world.step(dt);
        exporter.writeFrame(frame++, world.getBodies());
        if (!box->isAwake()) break;
    }
    Logger::info("Box is sleeping on static platform. Changing platform to Dynamic...");

    // 平台变动态
    platform->setType(BodyType::Dynamic, 1.0f);

    for (int i = 0; i < 120; ++i) {
        world.step(dt);
        exporter.writeFrame(frame++, world.getBodies());
    }

    if (box->isAwake() && platform->getVelocity().getY() < 0) {
        Logger::info("SUCCESS: Platform fell and box woke up!");
    }
    else {
        Logger::error("FAILURE: Static to Dynamic conversion failed to trigger simulation.");
    }
}

// --- 测试 3: 零漂移压力测试 ---
void TestZeroDriftStress() {
    Logger::info("Starting Test 3: 500 Bodies Zero-Drift Stress Test...");
    CSVExporter exporter("output/v2_009_003.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 放置 500 个方块
    for (int i = 0; i < 500; ++i) {
        Body* b = new Body(new Box(1, 1), (float)(i % 20), (float)(i / 20) + 5, 1.0f);
        b->forceSleep(); // 强行进入深度睡眠
        world.addBody(b);
    }

    // 重置宽相计数器
    world.getBroadPhase().m_moveCount = 0;
    Vector2 initialPos = world.getBodies()[0]->getPosition();

    Logger::info("Running 1000 frames of silent simulation...");
    for (int frame = 0; frame < 1000; ++frame) {
        world.step(dt);
        if (frame % 200 == 0) exporter.writeFrame(frame, world.getBodies());
    }

    Vector2 finalPos = world.getBodies()[0]->getPosition();
    float drift = (finalPos - initialPos).length();
    int moves = world.getBroadPhase().m_moveCount;

    Logger::info("Stress Test Result: Drift=" + std::to_string(drift) + ", BroadPhase Moves=" + std::to_string(moves));

    if (drift == 0.0f && moves == 0) {
        Logger::info("SUCCESS: Zero drift, Zero CPU overhead for BroadPhase sync!");
    }
    else {
        Logger::warning("Minor drift detected. Check your interceptor logic.");
    }
}

//int main() {
//    TestSuspensionWakeup();
//    TestTypeConversion();
//    TestZeroDriftStress();
//    return 0;
//}