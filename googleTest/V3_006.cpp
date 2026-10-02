// GoogleTest 版 V3/006：接触回调测试
// 移植自 tests/V3/006.cpp，场景逻辑原样保留，断言改为 gtest 宏
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "ContactListener.h"
#include "Box.h"
#include "Circle.h"
#include "Logger.h"
#include <cmath>

// 计数器监听器：记录各回调触发次数，并保存最后一次碰撞事件数据
class MockContactListener : public ContactListener {
public:
    int collisionEnter = 0;
    int collisionExit = 0;
    int triggerEnter = 0;
    int triggerExit = 0;
    CollisionEvent lastCollision;

    void onCollisionEnter(const CollisionEvent& event) override {
        collisionEnter++;
        lastCollision = event;
    }
    void onCollisionExit(const CollisionEvent& event) override {
        collisionExit++;
    }
    void onTriggerEnter(const TriggerEvent& event) override {
        triggerEnter++;
    }
    void onTriggerExit(const TriggerEvent& event) override {
        triggerExit++;
    }
};

// --- 场景 1: 经典碰撞三态回调捕获 ---
// 小球落向地面碰触并弹起：初碰帧恰好 1 次 onCollisionEnter，腾空帧恰好 1 次 onCollisionExit
TEST(V3_006, BounceCallback) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.addBody(new Body(new Box(20.0f, 2.0f), 0, -2.0f, 0.0f)); // 地面
    Body* ball = new Body(new Circle(0.5f), 0.0f, 1.0f, 1.0f);
    ball->getShape()->material.restitution = 1.0f; // 完全弹性：碰地即弹起
    world.addBody(ball);

    MockContactListener listener;
    world.setContactListener(&listener);

    // 球从 y=1 落下 1.5m 撞地（约 33 帧），弹起腾空（约 35 帧）。
    // 45 帧内只有一次起落，不会二次落地
    for (int i = 0; i < 45; ++i) world.step(dt);

    EXPECT_EQ(listener.collisionEnter, 1);
    EXPECT_EQ(listener.collisionExit, 1);

    // 回调中的法线严格垂直地面 (0,1)
    Vector2 n = listener.lastCollision.normal;
    EXPECT_NEAR(n.getX(), 0.0f, 1e-3f);
    EXPECT_NEAR(n.getY(), 1.0f, 1e-3f);

    // 接触点坐标准确：位于地面顶面 y=-1、球心正下方 x=0
    ASSERT_FALSE(listener.lastCollision.contacts.empty());
    Vector2 cp = listener.lastCollision.contacts[0];
    EXPECT_NEAR(cp.getX(), 0.0f, 1e-2f);
    EXPECT_NEAR(cp.getY(), -1.0f, 0.15f);
}

// --- 场景 2: 触发器与实体碰撞隔离 ---
// 小球穿过金币传感器（Trigger）：捕获 onTriggerEnter/onTriggerExit，
// 而 onCollisionEnter 计数严格为 0（绝对不串门）
TEST(V3_006, TriggerIsolation) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Box* sensor = new Box(10.0f, 1.0f);
    sensor->isTrigger = true;
    world.addBody(new Body(sensor, 0, 5.0f, 0.0f)); // 金币传感器，y 范围 4.5~5.5

    Body* ball = new Body(new Circle(0.5f), 0.0f, 8.0f, 1.0f);
    world.addBody(ball);

    MockContactListener listener;
    world.setContactListener(&listener);

    // 没有地面：球穿过传感器后持续下坠，全程不可能发生实体碰撞
    for (int i = 0; i < 90; ++i) world.step(dt);

    EXPECT_EQ(listener.triggerEnter, 1);
    EXPECT_EQ(listener.triggerExit, 1);
    EXPECT_EQ(listener.collisionEnter, 0);
    EXPECT_EQ(listener.collisionExit, 0);
}

// --- 场景 3: 死斗测试：回调内自毁刚体 ---
// 在 onCollisionEnter 内部直接调用 world->destroyBody(event.bodyB)，
// 引擎必须平稳执行（无内存访问崩溃），且下一帧刚体数正确减少 1
class DestroyListener : public ContactListener {
public:
    World* world = nullptr;
    int destroyCalls = 0;

    void onCollisionEnter(const CollisionEvent& event) override {
        destroyCalls++;
        world->destroyBody(event.bodyB); // 回调内自毁刚体！
    }
};

TEST(V3_006, SafeDestructionInCallback) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.addBody(new Body(new Box(20.0f, 2.0f), 0, -2.0f, 0.0f));
    Body* ball = new Body(new Circle(0.5f), 0.0f, 3.0f, 1.0f);
    world.addBody(ball);

    DestroyListener listener;
    listener.world = &world;
    world.setContactListener(&listener);

    int before = (int)world.getBodies().size(); // 2 (地面 + 球)

    // 球约 51 帧后初碰地面 -> 回调内销毁球；之后继续模拟，绝不能崩溃
    for (int i = 0; i < 90; ++i) world.step(dt);

    // 平稳执行完毕（走到这里即没有崩溃）；刚体数减少 1
    int after = (int)world.getBodies().size();
    EXPECT_EQ(listener.destroyCalls, 1);
    EXPECT_EQ(after, before - 1);
}
