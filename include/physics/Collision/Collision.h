#pragma once // 记得加上这个，防止重复包含
#include "Manifold.h"
#include <Capsule.h>
struct Projection {
    float min;
    float max;
};

class Collision
{
public:

    // 统一入口：根据 A 和 B 的类型自动选择算法
    static bool Dispatch(Manifold* m, Body* a, Body* b);

    static bool CircleVsCircle(Manifold* m, Body* a, Body* b);
    static bool BoxVsBox(Manifold* m, Body* a, Body* b);
    static bool CircleVsBox(Manifold* m, Body* circlebody, Body* boxbody);
	static bool AABBvsAABB(const AABB& a , const AABB& b);
    static bool CapsuleVsCircle(Manifold* m, Body* capsuleBody, Body* circleBody);
    static bool CapsuleVsCapsule(Manifold* m, Body* bodyA, Body* bodyB);
    static bool CapsuleVsBox(Manifold* m, Body* capsuleBody, Body* boxBody);
    // 点 P 到线段 [A, B] 的最近点（公开，供测试与外部使用）
    static Vector2 ClosestPointOnSegment(const Vector2& p, const Vector2& a, const Vector2& b);
private:
    static std::vector<Vector2> GetBoxWorldVertices(const Body* body);
    // 获取矩形在世界坐标系下的 X 轴方向（指向“右”侧）
    static Vector2 GetBodyAxisX(const Body* body) {
        float angle = body->GetRotation();
        return Vector2(cosf(angle), sinf(angle));
    }

    // 获取矩形在世界坐标系下的 Y 轴方向（指向“上”侧）
    static Vector2 GetBodyAxisY(const Body* body) {
        float angle = body->GetRotation();
        return Vector2(-sinf(angle), cosf(angle));
    }

    static Projection GetProjection(const std::vector<Vector2>& vertices, const Vector2& axis);

    static float GetOverlap(float minA, float maxA, float minB, float maxB);
    static bool Inside(Vector2 localPos, float hw, float hh) {
        // 如果 X 坐标在 [-hw, hw] 之间，且 Y 坐标在 [-hh, hh] 之间
        return std::abs(localPos.getX()) <= hw && std::abs(localPos.getY()) <= hh;
    }
	static void FindIncidentEdge(Vector2 out[ 2 ] , const std::vector<Vector2>& vertices , Vector2 normal);
	static bool IsPointInBody(Vector2 p , Body* b);

    static void ClosestPointsBetweenSegments(const Vector2& a, const Vector2& b,
        const Vector2& c, const Vector2& d,
        Vector2& outP1, Vector2& outP2);

    static void GetCapsuleWorldSegment(Body* body, Capsule* cap, Vector2& outA, Vector2& outB);
};

