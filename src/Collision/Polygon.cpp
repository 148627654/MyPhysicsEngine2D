#include "Polygon.h"

bool Polygon::Set(Vector2* vertices, int count)
{
    // 1. 顶点数量校验 (3 到 8 个点)
    if (count < 3 || count > 8) {
        return false;
    }

    // 2. 严格逆时针 (CCW) 与 凸性校验 (检查所有 count 个拐角)
    for (int i = 0; i < count; ++i) {
        int i0 = i;
        int i1 = (i + 1) % count;
        int i2 = (i + 2) % count;

        Vector2 edge1 = vertices[i1] - vertices[i0];
        Vector2 edge2 = vertices[i2] - vertices[i1];

        // 叉积 <= 0 说明：顺时针、平角(180度共线) 或 内凹(大于180度)
        float cross = Vector2::Cross(edge1, edge2);
        if (cross <= 0.0f) {
            return false;
        }
    }

    m_count = count;

    // 3. 计算面积与质心 (利用原始顶点)
    float totalArea = 0.0f;
    Vector2 centroid(0.0f, 0.0f);

    for (int i = 0; i < m_count; ++i) {
        Vector2& p1 = vertices[i];
        Vector2& p2 = vertices[(i + 1) % m_count];

        float cross = Vector2::Cross(p1, p2);
        totalArea += cross;
        centroid += (p1+p2) * cross;
    }

    totalArea *= 0.5f;
    if (std::abs(totalArea) < 1e-6f) {
        return false; // 面积过小退化
    }

    // 质心公式归一化: C = sum / (6 * Area)
    centroid = centroid * (1.0f / (6.0f * totalArea));
    m_area = std::abs(totalArea);

    // 4. 关键步骤：质心归零化并存入多边形内部数组！
    for (int i = 0; i < m_count; ++i) {
        m_vertices[i] = vertices[i] - centroid; // 统一减去质心
    }

    // 5. 预计算所有边缘的外法线
    for (int i = 0; i < m_count; ++i) {
        Vector2 edge = m_vertices[(i + 1) % m_count] - m_vertices[i];
        // 逆时针下的外法线为 (dy, -dx) 并归一化
        Vector2 normal(edge.getY(), -edge.getX());
        m_normals[i] = normal.Normalize();
    }

    return true;
}

float Polygon::GetSweepRadius() const
{
    float maxDistSq = 0.0f;

    // 遍历所有顶点，求距离平方的最大值（先不开方，省 CPU）
    for (int i = 0; i < m_count; ++i) {
        float distSq = m_vertices[i].Dot(m_vertices[i]);
        if (distSq > maxDistSq) {
            maxDistSq = distSq;
        }
    }

    // 只在最后开一次方
    return std::sqrt(maxDistSq);
}

MassData Polygon::ComputeMass(float density)
{
    MassData massData;
    massData.mass = 0.0f;
    massData.inertia = 0.0f;

    // 顶点少于 3 个无法构成多边形
    if (m_count < 3 || density <= 0.0f) {
        return massData;
    }

    float totalArea = 0.0f;
    float totalInertia = 0.0f;

    // 遍历每一个微元三角形 (0, 0) -> (Pi) -> (Pi+1)
    for (int i = 0; i < m_count; ++i) {
        const Vector2& p1 = m_vertices[i];
        const Vector2& p2 = m_vertices[(i + 1) % m_count];

        // 2D 叉积: D = p1.x * p2.y - p1.y * p2.x
        float D = Vector2::Cross(p1, p2);
        float triangleArea = 0.5f * D;
        totalArea += triangleArea;

        // 三角形二次惯量积分项
        float intx = p1.getX() * p1.getX() + p1.getX() * p2.getX() + p2.getX() * p2.getX();
        float inty = p1.getY() * p1.getY() + p1.getY() * p2.getY() + p2.getY() * p2.getY();

        // D / 12 * (intx + inty)
        totalInertia += (1.0f / 12.0f) * D * (intx + inty);
    }

    // 保证为正数
    totalArea = std::abs(totalArea);
    totalInertia = std::abs(totalInertia);

    // 1. 质量 = 面积 * 密度
    massData.mass = totalArea * density;

    // 2. 转动惯量 = 惯性积分 * 密度
    massData.inertia = totalInertia * density;

    return massData;
}

float Polygon::getArea() const
{
    // 如果已经在 Set() 中计算并缓存了 m_area，直接返回即可（O(1)性能最佳）
    if (m_count < 3) {
        return 0.0f;
    }

    // 如果想重新基于当前 m_vertices 动态算，你的写法完全正确：
    float doubleArea = 0.0f;
    for (int i = 0; i < m_count; ++i) {
        const Vector2& p1 = m_vertices[i];
        const Vector2& p2 = m_vertices[(i + 1) % m_count];
        doubleArea += Vector2::Cross(p1, p2);
    }

    return 0.5f * std::abs(doubleArea);
}

AABB Polygon::ComputeAABB(Vector2 pos, float angle)
{
    // 如果没有顶点，返回以当前位置为中心的零大小包围盒
    if (m_count == 0) {
        return AABB(pos, pos);
    }

    float cosA = std::cos(angle);
    float sinA = std::sin(angle);

    // 局部顶点到世界顶点的仿射变换：旋转 + 平移
    auto TransformToWorld = [&](const Vector2& v) -> Vector2 {
        return Vector2(
            pos.getX() + (v.getX() * cosA - v.getY() * sinA),
            pos.getY() + (v.getX() * sinA + v.getY() * cosA)
        );
        };

    // 用第 0 个世界顶点初始化 min 和 max
    Vector2 firstWorldV = TransformToWorld(m_vertices[0]);
    float minX = firstWorldV.getX();
    float maxX = firstWorldV.getX();
    float minY = firstWorldV.getY();
    float maxY = firstWorldV.getY();

    // 遍历剩余顶点，更新边界极值
    for (int i = 1; i < m_count; ++i) {
        Vector2 worldV = TransformToWorld(m_vertices[i]);

        minX = std::min(minX, worldV.getX());
        maxX = std::max(maxX, worldV.getX());
        minY = std::min(minY, worldV.getY());
        maxY = std::max(maxY, worldV.getY());
    }

    // 构造世界 AABB 返回
    return AABB(Vector2(minX, minY), Vector2(maxX, maxY));
}

bool Polygon::RayCast(RayCastOutput* output, RayCastInput& input,
    const Vector2& position, float rotation)
{
    if (m_count < 3) {
        return false;
    }

    // 1. 将射线起点和终点变换到多边形局部坐标系（平移 + 逆旋转）
    float cosA = std::cos(-rotation);
    float sinA = std::sin(-rotation);

    Vector2 relP1 = input.p1 - position;
    Vector2 relP2 = input.p2 - position;

    Vector2 p1(relP1.getX() * cosA - relP1.getY() * sinA, relP1.getX() * sinA + relP1.getY() * cosA);
    Vector2 p2(relP2.getX() * cosA - relP2.getY() * sinA, relP2.getX() * sinA + relP2.getY() * cosA);
    Vector2 d = p2 - p1; // 局部射线方向

    float lower = 0.0f;
    float upper = input.maxFraction;
    int hitIndex = -1;

    // 2. 遍历所有边，执行 Cyrus-Beck 半空间裁剪
    for (int i = 0; i < m_count; ++i) {
        // 分子: (V_i - p1) · n_i
        // 几何含义: p1 到该边所在直线的有向距离
        float numerator = m_normals[i].Dot(m_vertices[i] - p1);

        // 分母: d · n_i
        // 几何含义: 射线沿着外法线方向的分量投影
        float denominator = m_normals[i].Dot(d);

        // 情况 A: 射线平行于当前边
        if (std::abs(denominator) < 1e-6f) {
            // 如果起点在线段外侧，射线永远进不去多边形，直接未命中
            if (numerator < 0.0f) {
                return false;
            }
        }
        // 情况 B: 射线从外向内穿入该边 (入边: denominator < 0)
        else if (denominator < 0.0f) {
            float t = numerator / denominator;
            if (t > lower) {
                lower = t;
                hitIndex = i; // 记录最晚进入的边索引
            }
        }
        // 情况 C: 射线从内向外穿出该边 (出边: denominator > 0)
        else if (denominator > 0.0f) {
            float t = numerator / denominator;
            if (t < upper) {
                upper = t;
            }
        }

        // 裁剪剪枝：如果进入时间大于离开时间，说明射线未穿越多边形有效区域
        if (upper < lower) {
            return false;
        }
    }

    // 3. 检查是否发生合法命中
    if (hitIndex >= 0 && lower >= 0.0f && lower <= input.maxFraction) {
        output->fraction = lower;

        // 4. 将命中的局部外法线正向旋转回世界坐标系
        float worldCos = std::cos(rotation);
        float worldSin = std::sin(rotation);
        Vector2 localNormal = m_normals[hitIndex];

        output->normal.setX(localNormal.getX() * worldCos - localNormal.getY() * worldSin);
        output->normal.setY(localNormal.getX() * worldSin + localNormal.getY() * worldCos);

        return true;
    }

    return false;
}

