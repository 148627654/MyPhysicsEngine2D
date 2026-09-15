#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Box.h"
#include "../Collision/Circle.h"
#include "../Utils/Logger.h"
#include <cmath>

// --- 场景 1: 触发器穿透验证 ---
// 球从高处自由落体，穿过横在路径上的触发器盒子（无任何物理响应），最终落在地面
bool RunTriggerTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 地面 (density = 0 -> 静态)
    world.AddBody(new Body(new Box(20.0f, 2.0f), 0, -2.0f, 0.0f));

    // 触发器：静态盒子横在球的下落路径上 (y = 5，范围 4.5 ~ 5.5)
    Box* triggerBox = new Box(10.0f, 1.0f);
    triggerBox->isTrigger = true;
    world.AddBody(new Body(triggerBox, 0, 5.0f, 0.0f));

    // 动态球，从 y = 10 自由落体
    Body* ball = new Body(new Circle(0.5f), 0.0f, 10.0f, 1.0f);
    world.AddBody(ball);

    bool passedTrigger = false;
    float vyAtPass = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.Step(dt);
        float y = ball->GetPosition().getY();
        // 球心低于触发器底面 (4.5) 时记录速度
        if (!passedTrigger && y < 4.0f) {
            passedTrigger = true;
            vyAtPass = ball->GetVelocity().getY();
        }
    }
    float finalY = ball->GetPosition().getY();
    bool onGround = std::abs(finalY - (-0.5f)) < 0.3f; // 地面顶面 -1 + 球半径 0.5
    bool asleep = !ball->IsAwake(); // 静止稳定后应入睡

    Logger::Info("Trigger: passed=" + std::to_string(passedTrigger) +
        " vy_at_pass=" + std::to_string(vyAtPass) +
        " finalY=" + std::to_string(finalY) +
        " onGround=" + std::to_string(onGround) +
        " asleep=" + std::to_string(asleep));

    // 穿过触发器时仍在下落 (vy < 0，说明没有被弹起)，且最终停在路面并入睡
    return passedTrigger && onGround && vyAtPass < 0.0f && asleep;
}

// --- 场景 2: Material::Combine 四种合并模式数值验证 ---
bool RunCombineTest() {
    using namespace Physics2D;
    bool ok =
        std::abs(Material::Combine(0.5f, 0.5f, CombineMode::Average) - 0.5f) < 1e-6f &&
        std::abs(Material::Combine(0.2f, 0.8f, CombineMode::Minimum) - 0.2f) < 1e-6f &&
        std::abs(Material::Combine(0.25f, 1.0f, CombineMode::Multiply) - 0.5f) < 1e-6f &&
        std::abs(Material::Combine(0.2f, 0.8f, CombineMode::Maximum) - 0.8f) < 1e-6f;
    Logger::Info(std::string("CombineMode: ") + (ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: density 写回 material + 旧接口透传 + UpdateMassData ---
bool RunDensityTest() {
    // Body 构造时 density 应写回 shape->material（唯一数据源）
    Circle* c = new Circle(1.0f);
    Body* b = new Body(c, 0, 0, 2.5f);
    bool ok = std::abs(c->material.density - 2.5f) < 1e-6f;

    // 旧接口透传：setRestitution / setFriction 应写入 material
    b->setRestitution(0.4f);
    b->setFriction(0.7f);
    ok &= std::abs(c->material.restitution - 0.4f) < 1e-6f;
    ok &= std::abs(c->material.dynamicFriction - 0.7f) < 1e-6f;

    // 运行时修改材质密度后调用 UpdateMassData 重算质量属性
    c->material.density = 3.0f;
    b->UpdateMassData();

    Logger::Info(std::string("density write-back & delegate: ") + (ok ? "PASS" : "FAIL"));
    return ok;
}

//int main() {
//    Logger::Info(">>> Starting V3 001: Material & Trigger Test...");
//    bool ok = true;
//    ok &= RunTriggerTest();
//    ok &= RunCombineTest();
//    ok &= RunDensityTest();
//    Logger::Info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
//    return ok ? 0 : 1;
//}
