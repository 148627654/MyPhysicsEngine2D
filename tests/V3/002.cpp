#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Capsule.h"
#include "../Collision/Circle.h"
#include "../Collision/Box.h"
#include "../Collision/Collision.h"
#include "../Collision/Contact.h"
#include "../Utils/Logger.h"
#include <cmath>

// 向量近似相等判断
static bool Near(const Vector2& a, const Vector2& b, float tol) {
    return std::abs(a.getX() - b.getX()) < tol && std::abs(a.getY() - b.getY()) < tol;
}

// --- 场景 1: 几何质量计算 ---
// 胶囊体 r=1, L=4, 密度 ρ=1.0
// 面积 = 矩形 2r*L + 整圆 PI*r^2 = 8 + PI = 11.14159
bool RunMassTest() {
    Capsule* cap = new Capsule(1.0f, 4.0f);

    float expectedArea = 2.0f * 1.0f * 4.0f + Settings::PAI * 1.0f * 1.0f;
    bool ok = std::abs(cap->getArea() - expectedArea) < 1e-4f;
    Logger::Info("area = " + std::to_string(cap->getArea()) +
        " (expected " + std::to_string(expectedArea) + ") " + (ok ? "PASS" : "FAIL"));

    // 密度 1.0 -> mass = area, inertia = mass * (L^2/12 + r^2/2)
    Body* body = new Body(cap, 0.0f, 0.0f, 1.0f);
    float expectedMass = expectedArea * 1.0f;
    ok &= std::abs(body->getMass() - expectedMass) < 1e-3f;
    float expectedInertia = expectedMass * (4.0f * 4.0f / 12.0f + 1.0f * 1.0f / 2.0f);
    ok &= std::abs(body->getInertia() - expectedInertia) < 1e-3f;

    // UpdateMassData：运行时改密度后重算 mass 和 inertia
    cap->material.density = 2.0f;
    body->UpdateMassData();
    ok &= std::abs(body->getMass() - expectedMass * 2.0f) < 1e-3f;
    ok &= std::abs(body->getInertia() - expectedInertia * 2.0f) < 1e-3f;

    Logger::Info(std::string("Mass & UpdateMassData: ") + (ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 2: 点与线段最短距离 ---
bool RunClosestPointTest() {
    Vector2 a(0.0f, -2.0f), b(0.0f, 2.0f);

    // 测试点 (5, 0)：最近点必须是线段中点 (0, 0)
    Vector2 p1 = Collision::ClosestPointOnSegment(Vector2(5.0f, 0.0f), a, b);
    bool ok = Near(p1, Vector2(0.0f, 0.0f), 1e-4f);

    // 测试点 (3, 5)：最近点必须是端点 (0, 2)
    Vector2 p2 = Collision::ClosestPointOnSegment(Vector2(3.0f, 5.0f), a, b);
    ok &= Near(p2, Vector2(0.0f, 2.0f), 1e-4f);

    Logger::Info(std::string("ClosestPointOnSegment: ") + (ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: Capsule vs Circle 对撞反弹 ---
// 胶囊体平躺（旋转 90°）作静态目标，小球垂直落下击中圆柱侧翼
bool RunCapsuleVsCircleTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 平躺胶囊体：r=1, L=4, 中心 (0,0)，旋转 90° 后骨架沿 X 轴
    Body* capsule = new Body(new Capsule(1.0f, 4.0f), 0.0f, 0.0f, 0.0f);
    capsule->SetRotation(Settings::PAI / 2.0f);
    capsule->GetShape()->material.restitution = 1.0f;
    world.AddBody(capsule);

    // 小球从 (1.5, 4) 垂直落下，击中侧翼（非端点帽）
    Body* ball = new Body(new Circle(0.5f), 1.5f, 4.0f, 1.0f);
    ball->GetShape()->material.restitution = 1.0f;
    world.AddBody(ball);

    float startX = ball->GetPosition().getX();
    bool bounced = false;
    float maxVy = -1e9f;
    float minYAfterBounce = 1e9f;
    for (int i = 0; i < 240; ++i) {
        world.Step(dt);
        float vy = ball->GetVelocity().getY();
        maxVy = std::max(maxVy, vy);
        if (vy > 1.0f) bounced = true; // 获得向上的反弹速度
        if (bounced) minYAfterBounce = std::min(minYAfterBounce, ball->GetPosition().getY());
    }

    // 精准反弹（vy 接近撞击速度 ~7）、穿透被推开（未陷入胶囊内部）、垂直法线无横向偏移
    bool ok = bounced && maxVy > 3.0f
        && minYAfterBounce > 1.35f
        && std::abs(ball->GetPosition().getX() - startX) < 0.01f;

    Logger::Info("CapsuleVsCircle: bounced=" + std::to_string(bounced) +
        " maxVy=" + std::to_string(maxVy) +
        " minY=" + std::to_string(minYAfterBounce) +
        " dx=" + std::to_string(ball->GetPosition().getX() - startX) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 4: Capsule vs Capsule 十字交叉碰撞 ---
// 垂直胶囊体落在平躺胶囊体上，形成十字撞击
bool RunCapsuleVsCapsuleTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 平躺胶囊体（静态）
    Body* horizontal = new Body(new Capsule(1.0f, 4.0f), 0.0f, 0.0f, 0.0f);
    horizontal->SetRotation(Settings::PAI / 2.0f);
    world.AddBody(horizontal);

    // 垂直胶囊体从 (0, 5) 自由落下，中点正对十字交叉点
    Body* vertical = new Body(new Capsule(1.0f, 4.0f), 0.0f, 5.0f, 1.0f);
    world.AddBody(vertical);

    for (int i = 0; i < 300; ++i) world.Step(dt);

    // 检查接触流形：恰好 1 个接触点，法线竖直向上（从平躺胶囊指向垂直胶囊 = 受力反方向）
    bool ok = false;
    Vector2 normal;
    Vector2 contact;
    for (auto& pair : world.getContactMap()) {
        Contact* c = pair.second;
        if (!c->IsTouching()) continue;
        Manifold& mm = c->GetManifold();
        if (mm.contactCount == 1) {
            ok = true;
            normal = mm.normal;
            contact = mm.contacts[0];
        }
    }
    ok &= std::abs(normal.getX()) < 0.05f && normal.getY() > 0.95f;  // 法线竖直向上
    ok &= std::abs(contact.getX()) < 0.1f;                            // 接触点在两中点 x≈0
    ok &= std::abs(vertical->GetPosition().getY() - 4.0f) < 0.3f;     // 停在平躺胶囊上方
    ok &= std::abs(vertical->GetVelocity().getY()) < 0.01f;           // 已静止

    Logger::Info("CapsuleVsCapsule: contactCount=1 normal=(" + std::to_string(normal.getX()) +
        ", " + std::to_string(normal.getY()) + ") contactY=" + std::to_string(contact.getY()) +
        " restY=" + std::to_string(vertical->GetPosition().getY()) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

int main() {
    Logger::Info(">>> Starting V3 002: Capsule Test...");
    bool ok = true;
    ok &= RunMassTest();
    ok &= RunClosestPointTest();
    ok &= RunCapsuleVsCircleTest();
    ok &= RunCapsuleVsCapsuleTest();
    Logger::Info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
    return ok ? 0 : 1;
}
