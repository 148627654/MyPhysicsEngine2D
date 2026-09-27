#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Events/ContactListener.h"
#include "../Collision/Box.h"
#include "../Collision/Circle.h"
#include "../Utils/Logger.h"
#include <cmath>

// 计数器监听器：记录各回调触发次数，并保存最后一次碰撞事件数据
class MockContactListener : public ContactListener {
public:
    int collisionEnter = 0;
    int collisionExit = 0;
    int triggerEnter = 0;
    int triggerExit = 0;
    CollisionEvent lastCollision;

    void OnCollisionEnter(const CollisionEvent& event) override {
        collisionEnter++;
        lastCollision = event;
    }
    void OnCollisionExit(const CollisionEvent& event) override {
        collisionExit++;
    }
    void OnTriggerEnter(const TriggerEvent& event) override {
        triggerEnter++;
    }
    void OnTriggerExit(const TriggerEvent& event) override {
        triggerExit++;
    }
};

// --- 场景 1: 经典碰撞三态回调捕获 ---
// 小球落向地面碰触并弹起：初碰帧恰好 1 次 OnCollisionEnter，腾空帧恰好 1 次 OnCollisionExit
bool RunBounceCallbackTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.AddBody(new Body(new Box(20.0f, 2.0f), 0, -2.0f, 0.0f)); // 地面
    Body* ball = new Body(new Circle(0.5f), 0.0f, 1.0f, 1.0f);
    ball->GetShape()->material.restitution = 1.0f; // 完全弹性：碰地即弹起
    world.AddBody(ball);

    MockContactListener listener;
    world.setContactListener(&listener);

    // 球从 y=1 落下 1.5m 撞地（约 33 帧），弹起腾空（约 35 帧）。
    // 45 帧内只有一次起落，不会二次落地
    for (int i = 0; i < 45; ++i) world.Step(dt);

    bool ok = (listener.collisionEnter == 1) && (listener.collisionExit == 1);

    // 回调中的法线严格垂直地面 (0,1)
    Vector2 n = listener.lastCollision.normal;
    ok &= std::abs(n.getX()) < 1e-3f && std::abs(n.getY() - 1.0f) < 1e-3f;

    // 接触点坐标准确：位于地面顶面 y=-1、球心正下方 x=0
    ok &= !listener.lastCollision.contacts.empty();
    if (!listener.lastCollision.contacts.empty()) {
        Vector2 cp = listener.lastCollision.contacts[0];
        ok &= std::abs(cp.getX()) < 1e-2f && std::abs(cp.getY() - (-1.0f)) < 0.15f;
    }

    Logger::Info("BounceCallback: enter=" + std::to_string(listener.collisionEnter) +
        " exit=" + std::to_string(listener.collisionExit) +
        " n=(" + std::to_string(n.getX()) + "," + std::to_string(n.getY()) + ")" +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 2: 触发器与实体碰撞隔离 ---
// 小球穿过金币传感器（Trigger）：捕获 OnTriggerEnter/OnTriggerExit，
// 而 OnCollisionEnter 计数严格为 0（绝对不串门）
bool RunTriggerIsolationTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Box* sensor = new Box(10.0f, 1.0f);
    sensor->isTrigger = true;
    world.AddBody(new Body(sensor, 0, 5.0f, 0.0f)); // 金币传感器，y 范围 4.5~5.5

    Body* ball = new Body(new Circle(0.5f), 0.0f, 8.0f, 1.0f);
    world.AddBody(ball);

    MockContactListener listener;
    world.setContactListener(&listener);

    // 没有地面：球穿过传感器后持续下坠，全程不可能发生实体碰撞
    for (int i = 0; i < 90; ++i) world.Step(dt);

    bool ok = (listener.triggerEnter == 1) && (listener.triggerExit == 1)
        && (listener.collisionEnter == 0) && (listener.collisionExit == 0);

    Logger::Info("TriggerIsolation: triggerEnter=" + std::to_string(listener.triggerEnter) +
        " triggerExit=" + std::to_string(listener.triggerExit) +
        " collisionEnter=" + std::to_string(listener.collisionEnter) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: 死斗测试：回调内自毁刚体 ---
// 在 OnCollisionEnter 内部直接调用 world->DestroyBody(event.bodyB)，
// 引擎必须平稳执行（无内存访问崩溃），且下一帧刚体数正确减少 1
class DestroyListener : public ContactListener {
public:
    World* world = nullptr;
    int destroyCalls = 0;

    void OnCollisionEnter(const CollisionEvent& event) override {
        destroyCalls++;
        world->DestroyBody(event.bodyB); // 回调内自毁刚体！
    }
};

bool RunSafeDestructionTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.AddBody(new Body(new Box(20.0f, 2.0f), 0, -2.0f, 0.0f));
    Body* ball = new Body(new Circle(0.5f), 0.0f, 3.0f, 1.0f);
    world.AddBody(ball);

    DestroyListener listener;
    listener.world = &world;
    world.setContactListener(&listener);

    int before = (int)world.GetBodies().size(); // 2 (地面 + 球)

    // 球约 51 帧后初碰地面 -> 回调内销毁球；之后继续模拟，绝不能崩溃
    for (int i = 0; i < 90; ++i) world.Step(dt);

    // 平稳执行完毕（走到这里即没有崩溃）；刚体数减少 1
    int after = (int)world.GetBodies().size();
    bool ok = (listener.destroyCalls == 1) && (after == before - 1);

    Logger::Info("SafeDestruction: destroyCalls=" + std::to_string(listener.destroyCalls) +
        " bodies " + std::to_string(before) + "->" + std::to_string(after) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

int main() {
    Logger::Info(">>> Starting V3 006: Contact Callback Test...");
    bool ok = true;
    ok &= RunBounceCallbackTest();
    ok &= RunTriggerIsolationTest();
    ok &= RunSafeDestructionTest();
    Logger::Info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
    return ok ? 0 : 1;
}
