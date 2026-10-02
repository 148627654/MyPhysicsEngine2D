#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Dynamics/ContactManager.h"
#include "../Collision/Box.h"
#include "../Collision/Circle.h"
#include "../Utils/Logger.h"
#include <cmath>

// 统计本帧生命周期记录（可选按接触对过滤）
static void CountEvents(World& world, int& enter, int& stay, int& exit,
    Body* a = nullptr, Body* b = nullptr, bool* allTrigger = nullptr) {
    enter = stay = exit = 0;
    if (allTrigger) *allTrigger = true;
    for (const ContactRecord& r : world.getContactManager().getLifecycleRecords()) {
        // 按接触对过滤
        if (a && !((r.bodyA == a && r.bodyB == b) || (r.bodyA == b && r.bodyB == a))) continue;
        if (allTrigger && !r.isTrigger) *allTrigger = false;
        if (r.state == ContactState::Enter) enter++;
        else if (r.state == ContactState::Stay) stay++;
        else exit++;
    }
}

// --- 场景 1: 完整生命周期三连跳 (Enter -> Stay -> Exit) ---
bool RunLifecycleTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.addBody(new Body(new Box(20.0f, 2.0f), 0, -2.0f, 0.0f)); // 地面
    Body* ball = new Body(new Circle(0.5f), 0.0f, 3.0f, 1.0f);
    world.addBody(ball);

    bool ok = true;
    int enter, stay, exit;

    // 第 1 步：悬空下落，未碰触地面 -> 接触对数量为 0
    world.step(dt);
    ok &= (world.getContactMap().size() == 0);

    // 第 2 步：瞬移到刚好贴合地面（穿透 0）-> 捕获到唯一的 Enter
    ball->setPosition(0.0f, -0.5f);
    world.step(dt);
    CountEvents(world, enter, stay, exit);
    ok &= (enter == 1) && (stay == 0) && (exit == 0);

    // 第 3 步：连续模拟 3 帧 -> 每帧捕获 Stay，无 Enter/Exit
    int stayTotal = 0;
    for (int i = 0; i < 3; ++i) {
        world.step(dt);
        CountEvents(world, enter, stay, exit);
        ok &= (stay >= 1) && (enter == 0) && (exit == 0);
        stayTotal += stay;
    }
    ok &= (stayTotal == 3);

    // 第 4 步：瞬移到高空 -> 本帧捕获到 Exit
    ball->setPosition(0.0f, 10.0f);
    world.step(dt);
    CountEvents(world, enter, stay, exit);
    ok &= (exit == 1) && (enter == 0);

    // 第 5 步：继续模拟下一帧 -> 接触彻底清除，对数清零
    world.step(dt);
    CountEvents(world, enter, stay, exit);
    ok &= (world.getContactMap().size() == 0) && (enter == 0) && (exit == 0);

    Logger::info(std::string("Lifecycle Enter->Stay->Exit: ") + (ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 2: Trigger 传感器专属状态测试 ---
// 小球穿过 isTrigger 传感器：事件标记 isTrigger，且球未被任何冲量阻碍
bool RunTriggerEventTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Box* sensor = new Box(10.0f, 1.0f);
    sensor->isTrigger = true;
    world.addBody(new Body(sensor, 0, 5.0f, 0.0f)); // 静态传感器，y 范围 4.5 ~ 5.5

    Body* ball = new Body(new Circle(0.5f), 0.0f, 8.0f, 1.0f);
    world.addBody(ball);

    int enter = 0, stay = 0, exit = 0;
    bool allTrigger = true;
    float minVy = 1e9f;
    for (int i = 0; i < 120; ++i) {
        world.step(dt);
        minVy = std::min(minVy, ball->getVelocity().getY());
        int e, s, x;
        bool frameTrigger = true;
        CountEvents(world, e, s, x, nullptr, nullptr, &frameTrigger);
        if (!frameTrigger) allTrigger = false;
        enter += e; stay += s; exit += x;
    }

    // 穿过传感器：Enter 一次、Stay 多次、Exit 一次；全程事件 isTrigger == true；
    // 球持续加速下落（minVy 远小于 0），未被反弹阻碍
    bool ok = (enter == 1) && (stay >= 1) && (exit == 1) && allTrigger && (minVy < -8.0f);

    Logger::info("TriggerEvent: enter=" + std::to_string(enter) +
        " stay=" + std::to_string(stay) +
        " exit=" + std::to_string(exit) +
        " allTrigger=" + std::to_string(allTrigger) +
        " minVy=" + std::to_string(minVy) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: 多刚体接触互不干扰测试 ---
// 球 A 停在箱子 B 上（持续 Stay）；箱子 C 撞向 B 并在下一帧离开；
// A-B 的 Stay 完全不受 B-C 的 Enter/Exit 干扰
bool RunIsolationTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* boxB = new Body(new Box(2.0f, 2.0f), 0.0f, 0.0f, 0.0f); // 静态箱子 B，顶面 y=1
    world.addBody(boxB);
    Body* ballA = new Body(new Circle(0.5f), 0.0f, 1.5f, 1.0f);    // 球 A 停在 B 上
    world.addBody(ballA);

    // 箱子 C：动态触发器（零重力），初始远离，靠瞬移"撞向 B"
    Box* boxCShape = new Box(1.0f, 1.0f);
    boxCShape->isTrigger = true;
    Body* boxC = new Body(boxCShape, 20.0f, 0.0f, 1.0f);
    boxC->setGravityScale(0.0f);
    world.addBody(boxC);

    bool ok = true;
    int enter, stay, exit;

    // 阶段 1: 60 帧，A 停在 B 上 -> Enter 一次后持续 Stay
    int enterAB = 0, stayAB = 0;
    for (int i = 0; i < 60; ++i) {
        world.step(dt);
        CountEvents(world, enter, stay, exit, ballA, boxB);
        enterAB += enter; stayAB += stay;
    }
    ok &= (enterAB == 1) && (stayAB >= 50);

    // 阶段 2: C 突然撞向 B -> 本帧 B-C 捕获 Enter，同时 A-B 的 Stay 依然持续
    // C 半宽 0.5、B 半宽 1：C 中心放到 x=-1.4 时右面 -0.9，与 B 左面 -1 重叠 0.1
    boxC->setPosition(-1.4f, 0.0f);
    world.step(dt);
    int stayAB_f2, enterBC_f2;
    CountEvents(world, enter, stayAB_f2, exit, ballA, boxB);  // A-B 本帧
    CountEvents(world, enterBC_f2, stay, exit, boxB, boxC);   // B-C 本帧
    ok &= (stayAB_f2 >= 1) && (enterBC_f2 == 1);

    // 阶段 3: 下一帧 C 离开 -> B-C 捕获 Exit，A-B 的 Stay 依旧持续
    boxC->setPosition(20.0f, 0.0f);
    world.step(dt);
    int stayAB_f3, exitBC_f3;
    CountEvents(world, enter, stayAB_f3, exit, ballA, boxB);
    CountEvents(world, enter, stay, exitBC_f3, boxB, boxC);
    ok &= (stayAB_f3 >= 1) && (exitBC_f3 == 1);

    // 球 A 依旧停在 B 顶面上方
    ok &= std::abs(ballA->getPosition().getY() - 1.5f) < 0.1f;

    Logger::info("Isolation: enterAB=" + std::to_string(enterAB) +
        " stayAB=" + std::to_string(stayAB) +
        " enterBC=" + std::to_string(enterBC_f2) +
        " exitBC=" + std::to_string(exitBC_f3) +
        " A_y=" + std::to_string(ballA->getPosition().getY()) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

//int main() {
//    Logger::info(">>> Starting V3 005: Contact Event Test...");
//    bool ok = true;
//    ok &= RunLifecycleTest();
//    ok &= RunTriggerEventTest();
//    ok &= RunIsolationTest();
//    Logger::info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
//    return ok ? 0 : 1;
//}
