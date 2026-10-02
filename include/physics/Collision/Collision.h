#pragma once // 记得加上这个，防止重复包含
#include "Manifold.h"
#include <Capsule.h>
#include "Polygon.h"
#include <Collision.h>
struct Projection {
    float min;
    float max;
};

class Collision
{
public:

    // 统一入口：根据 A 和 B 的类型自动选择算法
    static bool dispatch(Manifold* m, Body* a, Body* b);

    static bool circleVsCircle(Manifold* m, Body* a, Body* b);
    static bool boxVsBox(Manifold* m, Body* a, Body* b);
    static bool circleVsBox(Manifold* m, Body* circlebody, Body* boxbody);
	static bool aabbVsAabb(const AABB& a , const AABB& b);
    static bool capsuleVsCircle(Manifold* m, Body* capsuleBody, Body* circleBody);
    static bool capsuleVsCapsule(Manifold* m, Body* bodyA, Body* bodyB);
    static bool capsuleVsBox(Manifold* m, Body* capsuleBody, Body* boxBody);
    // 点 P 到线段 [A, B] 的最近点（公开，供测试与外部使用）
    static Vector2 closestPointOnSegment(const Vector2& p, const Vector2& a, const Vector2& b);
    static bool polygonVsPolygon(Manifold* m, Body* a, Body* b);
    static bool polygonVsCircle(Manifold* m, Body* polyBody, Body* circleBody);
    static bool polygonVsCapsule(Manifold* m, Body* polyBody, Body* boxBody);

    // Sutherland-Hodgman 裁剪用的顶点结构
    struct ClipVertex {
        Vector2 v;
        float separation;
    };

    static void findIncidentEdge(ClipVertex out[2], const Polygon* incPoly, const Body* incBody,
        const Vector2& refNormal);

    
private:
    // SAT 核心：寻找 A 在 B 上的最大分离轴（最小穿透）
    static float findMaxSeparation(int& edgeIndex, const Polygon* polyA, const Body* bodyA,
        const Polygon* polyB, const Body* bodyB);

    // 用一条线段/半平面裁剪另一条线段
    static int clipSegmentToLine(ClipVertex vOut[2], const ClipVertex vIn[2],
        const Vector2& normal, float offset);

    static std::vector<Vector2> getBoxWorldVertices(const Body* body);
    // 获取矩形在世界坐标系下的 X 轴方向（指向“右”侧）
    static Vector2 getBodyAxisX(const Body* body) {
        float angle = body->getRotation();
        return Vector2(cosf(angle), sinf(angle));
    }

    // 获取矩形在世界坐标系下的 Y 轴方向（指向“上”侧）
    static Vector2 getBodyAxisY(const Body* body) {
        float angle = body->getRotation();
        return Vector2(-sinf(angle), cosf(angle));
    }

    static Projection getProjection(const std::vector<Vector2>& vertices, const Vector2& axis);

    static float getOverlap(float minA, float maxA, float minB, float maxB);
    static bool inside(Vector2 localPos, float hw, float hh) {
        // 如果 X 坐标在 [-hw, hw] 之间，且 Y 坐标在 [-hh, hh] 之间
        return std::abs(localPos.getX()) <= hw && std::abs(localPos.getY()) <= hh;
    }
	static void findIncidentEdge(Vector2 out[ 2 ] , const std::vector<Vector2>& vertices , Vector2 normal);
	static bool isPointInBody(Vector2 p , Body* b);

    static void closestPointsBetweenSegments(const Vector2& a, const Vector2& b,
        const Vector2& c, const Vector2& d,
        Vector2& outP1, Vector2& outP2);

    static void getCapsuleWorldSegment(Body* body, Capsule* cap, Vector2& outA, Vector2& outB);
};

