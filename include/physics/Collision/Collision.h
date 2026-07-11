#pragma once // 记得加上这个，防止重复包含
#include "Manifold.h"
struct Projection {
    float min;
    float max;
};

class Collision
{
public:
    static bool CircleVsCircle(Manifold* m, Body* a, Body* b);
    static bool BoxVsBox(Manifold* m, Body* a, Body* b);

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
};

