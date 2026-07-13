#include "Box.h"

AABB Box::ComputeAABB(Vector2 pos, float angle)
{
    float hw = getHalfWidth();
    float hh = getHalfHeigh();

    Vector2 Vertices[4];
    Vertices[0] = Vector2(-hw, hh);
    Vertices[1] = Vector2(hw, hh);
    Vertices[2] = Vector2(-hw, -hh);
    Vertices[3] = Vector2(hw, -hh);

    // 1. 先计算第一个点作为初始值，只计算一次旋转
    Vector2 v0 = pos + Vertices[0].Rotate(angle);
    float x_min = v0.getX();
    float x_max = x_min;
    float y_min = v0.getY();
    float y_max = y_min;

    // 2. 循环处理剩下的点
    for (int i = 1; i < 4; ++i)
    {
        // 关键：在这里只计算一次旋转，存入临时变量 vi
        Vector2 vi = pos + Vertices[i].Rotate(angle);
        float x = vi.getX();
        float y = vi.getY();

        // 3. 使用临时变量进行比较
        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }

    AABB aabb;
    aabb.min = Vector2(x_min, y_min);
    aabb.max = Vector2(x_max, y_max);
    return aabb;
}

MassData Box::ComputeMass(float density)
{
    MassData data;
    data.mass = density * getArea();
    data.inertia = data.mass / 12 * (width * width + height * height);
    return data;
}
