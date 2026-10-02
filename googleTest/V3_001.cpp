// GoogleTest 版 V3/001：Material & Trigger 测试
// 移植自 tests/V3/001.cpp，场景逻辑原样保留，断言改为 gtest 宏
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Box.h"
#include "Circle.h"
#include "Logger.h"
#include <cmath>

// --- 场景 1: 触发器穿透验证 ---
// 球从高处自由落体，穿过横在路径上的触发器盒子（无任何物理响应），最终落在地面
TEST(V3_001, TriggerPassthrough) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 地面 (density = 0 -> 静态)
    world.addBody(new Body(new Box(20.0f, 2.0f), 0, -2.0f, 0.0f));

    // 触发器：静态盒子横在球的下落路径上 (y = 5，范围 4.5 ~ 5.5)
    Box* triggerBox = new Box(10.0f, 1.0f);
    triggerBox->isTrigger = true;
    world.addBody(new Body(triggerBox, 0, 5.0f, 0.0f));

    // 动态球，从 y = 10 自由落体
    Body* ball = new Body(new Circle(0.5f), 0.0f, 10.0f, 1.0f);
    world.addBody(ball);

    bool passedTrigger = false;
    float vyAtPass = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        float y = ball->getPosition().getY();
        // 球心低于触发器底面 (4.5) 时记录速度
        if (!passedTrigger && y < 4.0f) {
            passedTrigger = true;
            vyAtPass = ball->getVelocity().getY();
        }
    }
    float finalY = ball->getPosition().getY();
    bool onGround = std::abs(finalY - (-0.5f)) < 0.3f; // 地面顶面 -1 + 球半径 0.5
    bool asleep = !ball->isAwake(); // 静止稳定后应入睡

    // 穿过触发器时仍在下落 (vy < 0，说明没有被弹起)，且最终停在路面并入睡
    EXPECT_TRUE(passedTrigger);
    EXPECT_TRUE(onGround);
    EXPECT_LT(vyAtPass, 0.0f);
    EXPECT_TRUE(asleep);
}

// --- 场景 2: Material::combine 四种合并模式数值验证 ---
TEST(V3_001, CombineModes) {
    using namespace Physics2D;
    EXPECT_NEAR(Material::combine(0.5f, 0.5f, CombineMode::Average), 0.5f, 1e-6f);
    EXPECT_NEAR(Material::combine(0.2f, 0.8f, CombineMode::Minimum), 0.2f, 1e-6f);
    EXPECT_NEAR(Material::combine(0.25f, 1.0f, CombineMode::Multiply), 0.5f, 1e-6f);
    EXPECT_NEAR(Material::combine(0.2f, 0.8f, CombineMode::Maximum), 0.8f, 1e-6f);
}

// --- 场景 3: density 写回 material + 旧接口透传 + updateMassData ---
TEST(V3_001, DensityWriteBackAndDelegate) {
    // Body 构造时 density 应写回 shape->material（唯一数据源）
    Circle* c = new Circle(1.0f);
    Body* b = new Body(c, 0, 0, 2.5f);
    EXPECT_NEAR(c->material.density, 2.5f, 1e-6f);

    // 旧接口透传：setRestitution / setFriction 应写入 material
    b->setRestitution(0.4f);
    b->setFriction(0.7f);
    EXPECT_NEAR(c->material.restitution, 0.4f, 1e-6f);
    EXPECT_NEAR(c->material.dynamicFriction, 0.7f, 1e-6f);

    // 运行时修改材质密度后调用 updateMassData 重算质量属性
    c->material.density = 3.0f;
    b->updateMassData();
    EXPECT_NEAR(c->material.density, 3.0f, 1e-6f);
}
