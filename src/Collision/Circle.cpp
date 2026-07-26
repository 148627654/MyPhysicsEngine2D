#include "Circle.h"




AABB Circle::ComputeAABB(Vector2 pos, float angle)
{
    AABB aabb;
    aabb.max = pos + Vector2(radius, radius);
    aabb.min = pos - Vector2(radius, radius);
    return aabb;
}

MassData Circle::ComputeMass(float density)
{
    MassData data;
    data.mass= density * getArea();
    data.inertia = data.mass / 2 * radius * radius;
    return data;
}

#include <cmath>

bool Circle::RayCast(RayCastOutput* output, RayCastInput& input,
    const Vector2& position, float rotation)
{
    // 射线矢量 d = p2 - p1
    Vector2 d = input.p2 - input.p1;
    // 从圆心指向射线起点的矢量 s = p1 - C
    Vector2 s = input.p1 - position;

    // 二次方程系数: at^2 + bt + c = 0
    float a = d.Dot(d); // d·d
    float b = 2.0f * s.Dot(d); // 2 * (s·d)
    float c = s.Dot(s) - radius * radius; // s·s - r^2

    // 计算判别式 Delta = b^2 - 4ac
    float sigma = b * b - 4.0f * a * c;

    // 1. 如果判别式小于 0，射线与圆不相交
    if (sigma < 0.0f || a < 1e-9f)
    {
        return false;
    }

    // 2. 求出较小的那个根（射线进入圆的撞击点）
    float t = (-b - std::sqrt(sigma)) / (2.0f * a);

    // 3. 判断撞击点是否在有效范围内 [0, maxFraction]
    if (t >= 0.0f && t <= input.maxFraction)
    {
        output->fraction = t;

        // 计算撞击点在世界空间的位置: P = p1 + t * d
        Vector2 hitPoint = input.p1 + d * t;

        // 计算圆表面法线: (P - C) / radius
        Vector2 normal = (hitPoint - position);

        // 归一化法线
        float len = std::sqrt(normal.Dot(normal));
        if (len > 1e-9f)
        {
            output->normal.setX(normal.getX() / len);
            output->normal.setY(normal.getY() / len);
        }
        else
        {
            output->normal.setX(0.0f);
            output->normal.setY(0.0f);
        }

        return true;
    }

    return false;
}
