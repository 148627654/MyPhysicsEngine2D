#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Collision/Polygon.h"
#include "../Collision/Circle.h"
#include "../Collision/Collision.h"
#include "../Collision/Contact.h"
#include "../Utils/Logger.h"
#include <cmath>

// 向量近似相等判断
static bool Near(const Vector2& a, const Vector2& b, float tol) {
    return std::abs(a.getX() - b.getX()) < tol && std::abs(a.getY() - b.getY()) < tol;
}

// 构造一个 2x2 矩形多边形 Body（静态，仅用于窄相测试）
static Body* MakeRectBody(float x, float y, float rot = 0.0f) {
    Vector2 rect[4] = {
        Vector2(-1.0f, -1.0f), Vector2(1.0f, -1.0f),
        Vector2(1.0f, 1.0f), Vector2(-1.0f, 1.0f)
    };
    Polygon* p = new Polygon();
    p->Set(rect, 4);
    Body* b = new Body(p, x, y, 0.0f);
    if (rot != 0.0f) b->SetRotation(rot);
    return b;
}

// --- 场景 1: 面面接触 2 接触点稳定测试（核心大考） ---
// 下方矩形 (0,0)，上方矩形 (0,1.95) 压下，穿透 0.05
bool RunFaceFaceTest() {
    Body* bottom = MakeRectBody(0.0f, 0.0f);
    Body* top = MakeRectBody(0.0f, 1.95f);

    Manifold m(bottom, top);
    bool hit = Collision::PolygonVsPolygon(&m, bottom, top);

    // contactCount 严格等于 2；两接触点 Y 相同（水平接触线段）；法线严格 (0,1)
    bool ok = hit
        && m.contactCount == 2
        && std::abs(m.contacts[0].getY() - m.contacts[1].getY()) < 1e-4f
        && std::abs(m.normal.getX()) < 1e-4f && std::abs(m.normal.getY() - 1.0f) < 1e-4f
        && std::abs(m.penetration - 0.05f) < 1e-3f;

    Logger::Info("FaceFace: hit=" + std::to_string(hit) +
        " count=" + std::to_string(m.contactCount) +
        " c0=(" + std::to_string(m.contacts[0].getX()) + "," + std::to_string(m.contacts[0].getY()) + ")" +
        " c1=(" + std::to_string(m.contacts[1].getX()) + "," + std::to_string(m.contacts[1].getY()) + ")" +
        " n=(" + std::to_string(m.normal.getX()) + "," + std::to_string(m.normal.getY()) + ")" +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 2: 角面接触 1 接触点测试 ---
// 上方矩形旋转 45°（尖角朝下）压在水平矩形上，接触点必须是尖角顶点
bool RunCornerFaceTest() {
    Body* bottom = MakeRectBody(0.0f, 0.0f);
    // 菱形最低点世界 y = center - sqrt(2)；让尖角扎进顶面 0.05
    float centerY = 1.0f + std::sqrt(2.0f) - 0.05f;
    Body* diamond = MakeRectBody(0.0f, centerY, Settings::PAI / 4.0f);

    Manifold m(bottom, diamond);
    bool hit = Collision::PolygonVsPolygon(&m, bottom, diamond);

    // contactCount 严格等于 1；接触点位于尖角顶点 (0, 0.95)；法线竖直向上
    bool ok = hit
        && m.contactCount == 1
        && std::abs(m.contacts[0].getX()) < 1e-3f
        && std::abs(m.contacts[0].getY() - 0.95f) < 1e-2f
        && std::abs(m.normal.getX()) < 1e-4f && std::abs(m.normal.getY() - 1.0f) < 1e-4f;

    Logger::Info("CornerFace: hit=" + std::to_string(hit) +
        " count=" + std::to_string(m.contactCount) +
        " c0=(" + std::to_string(m.contacts[0].getX()) + "," + std::to_string(m.contacts[0].getY()) + ")" +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: 错位重叠裁剪测试 ---
// 上方矩形向右平移半个身位 (1, 1.95)，接触点必须被裁剪限制在重叠交集 x∈[0,1] 内
bool RunClippedOverlapTest() {
    Body* bottom = MakeRectBody(0.0f, 0.0f);
    Body* shifted = MakeRectBody(1.0f, 1.95f);

    Manifold m(bottom, shifted);
    bool hit = Collision::PolygonVsPolygon(&m, bottom, shifted);

    float minX = std::min(m.contacts[0].getX(), m.contacts[1].getX());
    float maxX = std::max(m.contacts[0].getX(), m.contacts[1].getX());

    // 2 个接触点，都落在重叠线段 x∈[0,1] 内，没有点溢出到空中
    bool ok = hit
        && m.contactCount == 2
        && minX >= -1e-3f && maxX <= 1.0f + 1e-3f
        && std::abs(m.contacts[0].getY() - m.contacts[1].getY()) < 1e-4f
        && std::abs(m.normal.getX()) < 1e-4f && std::abs(m.normal.getY() - 1.0f) < 1e-4f;

    Logger::Info("ClippedOverlap: count=" + std::to_string(m.contactCount) +
        " c0=(" + std::to_string(m.contacts[0].getX()) + "," + std::to_string(m.contacts[0].getY()) + ")" +
        " c1=(" + std::to_string(m.contacts[1].getX()) + "," + std::to_string(m.contacts[1].getY()) + ")" +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 4: Polygon vs Circle 斜面接触测试 ---
// 小球垂直落在 45° 斜面上，法线必须精准垂直于斜面并成功反弹
bool RunPolygonVsCircleTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 45° 斜面：矩形多边形旋转 45°，顶面变成斜坡
    Body* slope = MakeRectBody(0.0f, 0.0f, Settings::PAI / 4.0f);
    world.AddBody(slope);

    // 小球从斜面中点 (-0.7071, 0.7071) 正上方落下
    Body* ball = new Body(new Circle(0.5f), -0.7071f, 2.0f, 1.0f);
    ball->GetShape()->material.restitution = 1.0f;
    world.AddBody(ball);

    Vector2 faceNormal(-0.7071f, 0.7071f); // 局部 (0,1) 旋转 45° 后的世界法线
    Vector2 faceDir(0.7071f, 0.7071f);     // 斜面方向（沿面）

    bool foundContact = false;
    Vector2 contactNormal;
    float minVx = 1e9f;
    for (int i = 0; i < 150; ++i) {
        world.Step(dt);
        minVx = std::min(minVx, ball->GetVelocity().getX());
        for (auto& pair : world.getContactMap()) {
            Contact* c = pair.second;
            if (!c->IsTouching()) continue;
            Manifold& mm = c->GetManifold();
            if (mm.contactCount >= 1) {
                foundContact = true;
                contactNormal = mm.normal;
            }
        }
    }

    // 法线垂直于斜面（与面方向点积为 0）且方向正确；球沿法线反射被弹走（vx 变为负）
    bool ok = foundContact
        && Near(contactNormal, faceNormal, 0.02f)
        && std::abs(contactNormal.Dot(faceDir)) < 1e-3f
        && minVx < -1.5f;

    Logger::Info("PolygonVsCircle: contact=" + std::to_string(foundContact) +
        " n=(" + std::to_string(contactNormal.getX()) + "," + std::to_string(contactNormal.getY()) + ")" +
        " dot(n,face)=" + std::to_string(contactNormal.Dot(faceDir)) +
        " minVx=" + std::to_string(minVx) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}
//
//int main() {
//    Logger::Info(">>> Starting V3 004: Polygon Collision Test...");
//    bool ok = true;
//    ok &= RunFaceFaceTest();
//    ok &= RunCornerFaceTest();
//    ok &= RunClippedOverlapTest();
//    ok &= RunPolygonVsCircleTest();
//    Logger::Info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
//    return ok ? 0 : 1;
//}
