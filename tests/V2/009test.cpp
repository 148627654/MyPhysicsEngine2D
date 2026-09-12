#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"
#include "../Utils/CSVExporter.h"

// --- 测试 1: 悬空唤醒测试 ---
void TestSuspensionWakeup() {
    Logger::Info("Starting Test 1: Suspension Wakeup...");
    CSVExporter exporter("output/v2_009_001.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* ground = new Body(new Box(20, 2), 0, -1, 0); world.AddBody(ground);
    Body* box1 = new Body(new Box(1, 1), 0, 0.5f, 1.0f); world.AddBody(box1);
    Body* box2 = new Body(new Box(1, 1), 0, 1.5f, 1.0f); world.AddBody(box2);

    int frame = 0;
    // 1. 等待入睡
    while (frame < 500) {
        world.Step(dt);
        exporter.WriteFrame(frame++, world.GetBodies());
        if (!box1->IsAwake() && !box2->IsAwake()) break;
    }
    Logger::Info("Boxes are sleeping. Now removing box1 (the base)...");

    // 2. 移除底座
    world.RemoveBody(box1);

    // 3. 观察 box2 是否被唤醒
    for (int i = 0; i < 60; ++i) {
        world.Step(dt);
        exporter.WriteFrame(frame++, world.GetBodies());
    }

    if (box2->IsAwake() && box2->GetPosition().getY() < 1.4f) {
        Logger::Info("SUCCESS: Box2 was awakened and fell down!");
    }
    else {
        Logger::Error("FAILURE: Box2 is still floating!");
    }
}

// --- 测试 2: 类型转换测试 ---
void TestTypeConversion() {
    Logger::Info("Starting Test 2: Type Conversion Wakeup...");
    CSVExporter exporter("output/v2_009_002.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 创建一个静态平台
    Body* platform = new Body(new Box(5, 1), 0, 0, 0); world.AddBody(platform);
    Body* box = new Body(new Box(1, 1), 0, 1.0f, 1.0f); world.AddBody(box);

    int frame = 0;
    while (frame < 300) {
        world.Step(dt);
        exporter.WriteFrame(frame++, world.GetBodies());
        if (!box->IsAwake()) break;
    }
    Logger::Info("Box is sleeping on static platform. Changing platform to Dynamic...");

    // 平台变动态
    platform->SetType(BodyType::Dynamic, 1.0f);

    for (int i = 0; i < 120; ++i) {
        world.Step(dt);
        exporter.WriteFrame(frame++, world.GetBodies());
    }

    if (box->IsAwake() && platform->GetVelocity().getY() < 0) {
        Logger::Info("SUCCESS: Platform fell and box woke up!");
    }
    else {
        Logger::Error("FAILURE: Static to Dynamic conversion failed to trigger simulation.");
    }
}

// --- 测试 3: 零漂移压力测试 ---
void TestZeroDriftStress() {
    Logger::Info("Starting Test 3: 500 Bodies Zero-Drift Stress Test...");
    CSVExporter exporter("output/v2_009_003.csv");
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 放置 500 个方块
    for (int i = 0; i < 500; ++i) {
        Body* b = new Body(new Box(1, 1), (float)(i % 20), (float)(i / 20) + 5, 1.0f);
        b->ForceSleep(); // 强行进入深度睡眠
        world.AddBody(b);
    }

    // 重置宽相计数器
    world.GetBroadPhase().m_moveCount = 0;
    Vector2 initialPos = world.GetBodies()[0]->GetPosition();

    Logger::Info("Running 1000 frames of silent simulation...");
    for (int frame = 0; frame < 1000; ++frame) {
        world.Step(dt);
        if (frame % 200 == 0) exporter.WriteFrame(frame, world.GetBodies());
    }

    Vector2 finalPos = world.GetBodies()[0]->GetPosition();
    float drift = (finalPos - initialPos).Length();
    int moves = world.GetBroadPhase().m_moveCount;

    Logger::Info("Stress Test Result: Drift=" + std::to_string(drift) + ", BroadPhase Moves=" + std::to_string(moves));

    if (drift == 0.0f && moves == 0) {
        Logger::Info("SUCCESS: Zero drift, Zero CPU overhead for BroadPhase sync!");
    }
    else {
        Logger::Warning("Minor drift detected. Check your interceptor logic.");
    }
}

//int main() {
//    TestSuspensionWakeup();
//    TestTypeConversion();
//    TestZeroDriftStress();
//    return 0;
//}