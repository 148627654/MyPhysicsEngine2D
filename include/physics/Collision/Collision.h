#pragma once // 记得加上这个，防止重复包含
#include "Manifold.h"
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
};

