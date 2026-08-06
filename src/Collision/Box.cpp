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

bool Box::RayCast(RayCastOutput* output, RayCastInput& input,
    const Vector2& position, float rotation)
{
    // 1. 将射线转换到方块的局部坐标系 (Inverse Transform)
    // 先平移，再旋转 (使用负角度进行逆旋转)
    float cosR = cos(-rotation);
    float sinR = sin(-rotation);

    // 局部空间起点 p1
    Vector2 p1 = input.p1 - position;
    Vector2 localP1;
    localP1.setX(p1.getX() * cosR - p1.getY() * sinR);
    localP1.setY(p1.getX() * sinR + p1.getY() * cosR);

    // 局部空间终点 p2
    Vector2 p2 = input.p2 - position;
    Vector2 localP2;
    localP2.setX(p2.getX() * cosR - p2.getY() * sinR);
    localP2.setY(p2.getX() * sinR + p2.getY() * cosR);
    // 射线方向矢量
    Vector2 d = localP2 - localP1;

    // 2. 在局部空间执行 AABB 射线检测
    // 局部空间的 AABB 范围是 [-halfWidth, -halfHeight] 到 [halfWidth, halfHeight]
    float hW = width * 0.5f;
    float hH = height * 0.5f;

    float tmin = -1.0f;
    float tmax = input.maxFraction;

    Vector2 normal(0, 0); // 用于记录撞击面的法线

    // 针对 X 和 Y 轴执行 Slab 算法
    // --- X 轴 ---
    if (std::abs(d.getX()) < 1e-6f) {
        if (localP1.getX() < -hW || localP1.getX() > hW) return false;
    }
    else {
        float t1 = (-hW - localP1.getX()) / d.getX();
        float t2 = (hW - localP1.getX()) / d.getX();

        Vector2 n1(-1.0f, 0.0f); // 左面法线
        Vector2 n2(1.0f, 0.0f);  // 右面法线

        if (t1 > t2) { std::swap(t1, t2); std::swap(n1, n2); }

        if (t1 > tmin) {
            tmin = t1;
            normal = n1; // 记录射线进入方块的那一面的法线
        }
        tmax = std::min(tmax, t2);
    }

    // --- Y 轴 ---
    if (std::abs(d.getY()) < 1e-6f) {
        if (localP1.getY() < -hH || localP1.getY() > hH) return false;
    }
    else {
        float t1 = (-hH - localP1.getY()) / d.getY();
        float t2 = (hH - localP1.getY()) / d.getY();

        Vector2 n1(0.0f, -1.0f); // 底面法线
        Vector2 n2(0.0f, 1.0f);  // 顶面法线

        if (t1 > t2) { std::swap(t1, t2); std::swap(n1, n2); }

        if (t1 > tmin) {
            tmin = t1;
            normal = n1;
        }
        tmax = std::min(tmax, t2);
    }

    // 3. 判定最终结果
    if (tmin >= 0.0f && tmin <= tmax) {
        output->fraction = tmin;

        // 将局部法线旋转回世界空间 (使用正角度)
        float cosW = cos(rotation);
        float sinW = sin(rotation);
        output->normal.setX(normal.getX() * cosW - normal.getY() * sinW);
        output->normal.setY(normal.getX() * sinW + normal.getY() * cosW);

        return true;
    }

    return false;
}

float Box::GetSweepRadius() const
{
    float hw = width * 0.5f;
    float hh = height * 0.5f;
    return std::sqrt(hw * hw + hh * hh);
}
