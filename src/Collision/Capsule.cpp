#include "Capsule.h"


MassData Capsule::ComputeMass(float density)
{
	MassData data;
	data.mass = density * getArea();
	data.inertia = data.mass * (m_length * m_length / 12.0f + m_radius * m_radius / 2.0f);
	return data;
}

float Capsule::getArea() const
{
	// 矩形部分 2r*L + 两个半圆合并成整圆 πr²
	return 2 * m_radius * m_length + Settings::PAI * m_radius * m_radius;
}

bool Capsule::RayCast(RayCastOutput* output, RayCastInput& input,
    const Vector2& position, float rotation)
{
    // --- 步骤 1：将射线变换到胶囊体的局部坐标系 ---
    float cosA = std::cos(-rotation);
    float sinA = std::sin(-rotation);

    Vector2 localP1_raw = input.p1 - position;
    Vector2 localP2_raw = input.p2 - position;

    // 逆旋转 (x*cos - y*sin, x*sin + y*cos)
    Vector2 p1(localP1_raw.getX() * cosA - localP1_raw.getY() * sinA,
        localP1_raw.getX() * sinA + localP1_raw.getY() * cosA);
    Vector2 p2(localP2_raw.getX() * cosA - localP2_raw.getY() * sinA,
        localP2_raw.getX() * sinA + localP2_raw.getY() * cosA);

    Vector2 d = p2 - p1; // 局部射线方向与长度
    float h = m_length * 0.5f;

    float minFraction = input.maxFraction;
    Vector2 localNormal(0.0f, 0.0f);
    bool hit = false;

    // --- 步骤 2：测试左侧线段 (x = -r, y in [-h, h]) ---
    if (std::abs(d.getX()) > 1e-6f) {
        float t = (-m_radius - p1.getX()) / d.getX();
        if (t >= 0.0f && t < minFraction) {
            float y = p1.getY() + t * d.getY();
            if (y >= -h && y <= h) {
                minFraction = t;
                localNormal = Vector2(-1.0f, 0.0f);
                hit = true;
            }
        }
    }

    // --- 步骤 3：测试右侧线段 (x = +r, y in [-h, h]) ---
    if (std::abs(d.getX()) > 1e-6f) {
        float t = (m_radius - p1.getX()) / d.getX();
        if (t >= 0.0f && t < minFraction) {
            float y = p1.getY() + t * d.getY();
            if (y >= -h && y <= h) {
                minFraction = t;
                localNormal = Vector2(1.0f, 0.0f);
                hit = true;
            }
        }
    }

    // --- 辅助函数：射线与圆相交测试 ---
    auto RayCastCircle = [&](const Vector2& center, bool isTopCap) {
        Vector2 m = p1 - center;
        float a = d.Dot(d);
        float b = 2.0f * m.Dot(d);
        float c = m.Dot(m) - m_radius * m_radius;

        float discriminant = b * b - 4.0f * a * c;
        if (discriminant >= 0.0f && a > 1e-6f) {
            float t = (-b - std::sqrt(discriminant)) / (2.0f * a);
            if (t >= 0.0f && t < minFraction) {
                float y = p1.getY() + t * d.getY();
                // 顶部半圆只接受 y >= h，底部半圆只接受 y <= -h
                if ((isTopCap && y >= h) || (!isTopCap && y <= -h)) {
                    minFraction = t;
                    Vector2 hitPoint = p1 + d * t;
                    localNormal = (hitPoint - center) * (1.0f / m_radius);
                    hit = true;
                }
            }
        }
        };

    // --- 步骤 4：测试顶部半圆与底部半圆 ---
    RayCastCircle(Vector2(0.0f, h), true);  // 顶部半圆
    RayCastCircle(Vector2(0.0f, -h), false); // 底部半圆

    // --- 步骤 5：如果命中，把法线旋转回世界坐标系 ---
    if (hit) {
        output->fraction = minFraction;

        // 正向旋转回去
        float worldCos = std::cos(rotation);
        float worldSin = std::sin(rotation);
        output->normal.setX(localNormal.getX() * worldCos - localNormal.getY() * worldSin);
        output->normal.setY(localNormal.getX() * worldSin + localNormal.getY() * worldCos);
        return true;
    }

    return false;
}

float Capsule::GetSweepRadius() const
{
	return m_length * 0.5f + m_radius;
}

AABB Capsule::ComputeAABB(Vector2 pos, float angle)
{
    float halfLength = m_length * 0.5f;

    // 计算朝向向量（局部 (0, 1) 旋转 angle 后的世界方向）
    // 旋转矩阵: x' = -sin(angle) * y, y' = cos(angle) * y
    float sinA = std::sin(angle);
    float cosA = std::cos(angle);
    Vector2 axis(-sinA * halfLength, cosA * halfLength);

    // 骨架线段的世界端点 P1, P2
    Vector2 p1 = pos - axis; // 下圆心
    Vector2 p2 = pos + axis; // 上圆心

    // 线段的包围盒
    float minX = std::min(p1.getX(), p2.getX());
    float minY = std::min(p1.getY(), p2.getY());
    float maxX = std::max(p1.getX(), p2.getX());
    float maxY = std::max(p1.getY(), p2.getY());

    // 四周向外扩张半径 r
    Vector2 minPoint(minX - m_radius, minY - m_radius);
    Vector2 maxPoint(maxX + m_radius, maxY + m_radius);

    return AABB(minPoint, maxPoint);
}