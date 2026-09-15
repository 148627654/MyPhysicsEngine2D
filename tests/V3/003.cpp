#include "../Dynamics/Body.h"
#include "../Collision/Polygon.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"
#include <cmath>

// 向量近似相等判断
static bool Near(const Vector2& a, const Vector2& b, float tol) {
    return std::abs(a.getX() - b.getX()) < tol && std::abs(a.getY() - b.getY()) < tol;
}

// 从内部顶点重算质心 (顶点已质心归零时结果应约为 (0,0))
static Vector2 ComputeCentroid(const Polygon& poly) {
    Vector2 centroid(0.0f, 0.0f);
    float doubleArea = 0.0f;
    int n = poly.GetVertexCount();
    for (int i = 0; i < n; ++i) {
        const Vector2& p1 = poly.GetVertex(i);
        const Vector2& p2 = poly.GetVertex((i + 1) % n);
        float c = Vector2::Cross(p1, p2);
        doubleArea += c;
        centroid += (p1 + p2) * c;
    }
    return centroid * (1.0f / (3.0f * doubleArea)); // sum/(6A) = sum/(3*doubleArea)
}

// --- 场景 1: Box 与 Polygon 的等价性对照 ---
// Box 宽高 (2,4)、密度 1.5；把 4 个角点喂给 Polygon，两者面积/质量/惯量必须一致
bool RunBoxPolygonEquivalenceTest() {
    Box box(2.0f, 4.0f);
    MassData boxMass = box.ComputeMass(1.5f); // 理论值: mass=12, inertia=20

    // 4 个矩形角点 (逆时针): (-1,-2) (1,-2) (1,2) (-1,2)
    Vector2 corners[4] = {
        Vector2(-1.0f, -2.0f), Vector2(1.0f, -2.0f),
        Vector2(1.0f, 2.0f), Vector2(-1.0f, 2.0f)
    };
    Polygon poly;
    bool setOk = poly.Set(corners, 4);

    // 断言 1: 面积严格等于 8.0，与 Box
    bool ok = setOk
        && std::abs(poly.getArea() - 8.0f) < 1e-5f
        && std::abs(poly.getArea() - box.getArea()) < 1e-5f;

    // 断言 2: 质心必须为 (0,0)（顶点已做质心归零平移）
    Vector2 centroid = ComputeCentroid(poly);
    ok &= centroid.Length() < 1e-4f;

    // 断言 3: ComputeMass 的质量与转动惯量和 Box 误差 < 1e-5
    MassData polyMass = poly.ComputeMass(1.5f);
    ok &= std::abs(polyMass.mass - boxMass.mass) < 1e-5f;
    ok &= std::abs(polyMass.inertia - boxMass.inertia) < 1e-5f;

    Logger::Info("Box vs Polygon: area=" + std::to_string(poly.getArea()) +
        " mass=" + std::to_string(polyMass.mass) +
        " inertia=" + std::to_string(polyMass.inertia) +
        " (box inertia=" + std::to_string(boxMass.inertia) + ")" +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 2: 质心平移修正 ---
// 偏心三角形 (10,0) (13,0) (10,4)，质心理论值 (11, 4/3)；Set() 后内部顶点应已平移使质心归零
bool RunCentroidShiftTest() {
    Vector2 tri[3] = {
        Vector2(10.0f, 0.0f), Vector2(13.0f, 0.0f), Vector2(10.0f, 4.0f)
    };
    Polygon poly;
    bool setOk = poly.Set(tri, 3);

    // 顶点平移后重算质心应归零
    Vector2 centroid = ComputeCentroid(poly);
    bool ok = setOk && centroid.Length() < 1e-4f;

    // 平移不改变面积: 底 3 高 4 的三角形面积 = 6
    ok &= std::abs(poly.getArea() - 6.0f) < 1e-5f;

    Logger::Info("CentroidShift: centroid=(" + std::to_string(centroid.getX()) +
        ", " + std::to_string(centroid.getY()) + ")" +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: 凸性与逆时针拦截 ---
// 顺时针顶点集、凹四边形（飞镖形）、非法顶点数都必须被 Set() 拒绝
bool RunValidationTest() {
    Polygon p1;
    Polygon p2;
    Polygon p3;

    // 顺时针矩形 (0,0)->(0,4)->(4,4)->(4,0)：拐角叉积为负
    Vector2 cw[4] = {
        Vector2(0.0f, 0.0f), Vector2(0.0f, 4.0f), Vector2(4.0f, 4.0f), Vector2(4.0f, 0.0f)
    };
    bool rejectCW = (p1.Set(cw, 4) == false);

    // 凹四边形（飞镖形）：(0.5,0.5) 凹进内部
    Vector2 dart[4] = {
        Vector2(0.0f, 0.0f), Vector2(2.0f, 0.0f), Vector2(0.5f, 0.5f), Vector2(0.0f, 2.0f)
    };
    bool rejectConcave = (p2.Set(dart, 4) == false);

    // 顶点数不足 3
    Vector2 two[2] = { Vector2(0.0f, 0.0f), Vector2(1.0f, 0.0f) };
    bool rejectFewVerts = (p3.Set(two, 2) == false);

    bool ok = rejectCW && rejectConcave && rejectFewVerts;
    Logger::Info("Validation: rejectCW=" + std::to_string(rejectCW) +
        " rejectConcave=" + std::to_string(rejectConcave) +
        " rejectFewVerts=" + std::to_string(rejectFewVerts) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 4: 多边形射线投射 ---
// 射线从外部水平射向正五边形（外接圆半径 2，顶点从 90 度起），应击中左侧边
bool RunRayCastTest() {
    const int N = 5;
    Vector2 verts[N];
    for (int i = 0; i < N; ++i) {
        float ang = (90.0f + 72.0f * i) * Settings::PAI / 180.0f;
        verts[i] = Vector2(2.0f * std::cos(ang), 2.0f * std::sin(ang));
    }
    Polygon poly;
    if (!poly.Set(verts, N)) {
        Logger::Info("RayCast: pentagon Set failed");
        return false;
    }

    RayCastInput input;
    input.p1 = Vector2(-5.0f, 0.0f);
    input.p2 = Vector2(5.0f, 0.0f);
    input.maxFraction = 1.0f;

    RayCastOutput output;
    bool hit = poly.RayCast(&output, input, Vector2(0.0f, 0.0f), 0.0f);

    // 理论值：击中左侧边（162 度与 234 度顶点之间），击中点 x = -1.70130
    // fraction = (5 - 1.70130) / 10 = 0.32987，法线指向 198 度方向
    float expectedT = 0.32987f;
    Vector2 expectedNormal(-0.95106f, -0.30902f);

    bool ok = hit
        && std::abs(output.fraction - expectedT) < 1e-3f
        && Near(output.normal, expectedNormal, 1e-3f);

    // 法线必须精准垂直于被击中边缘: n dot edgeDir = 0
    Vector2 hitEdge = verts[2] - verts[1]; // 边 1 -> 2
    float dot = output.normal.Dot(hitEdge);
    ok &= std::abs(dot) < 1e-4f;

    Logger::Info("RayCast: hit=" + std::to_string(hit) +
        " t=" + std::to_string(output.fraction) +
        " normal=(" + std::to_string(output.normal.getX()) + ", " + std::to_string(output.normal.getY()) + ")" +
        " dot(n,edge)=" + std::to_string(dot) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

int main() {
    Logger::Info(">>> Starting V3 003: Polygon Test...");
    bool ok = true;
    ok &= RunBoxPolygonEquivalenceTest();
    ok &= RunCentroidShiftTest();
    ok &= RunValidationTest();
    ok &= RunRayCastTest();
    Logger::Info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
    return ok ? 0 : 1;
}
