#include "AABB.h"
#include <algorithm>

AABB AABB::Union(const AABB& a, const AABB& b)
{
	AABB ans;
	ans.min.setX(std::min(a.min.getX(), b.min.getX()));
	ans.min.setY(std::min(a.min.getY(), b.min.getY()));

	ans.max.setX(std::max(a.max.getX(), b.max.getX()));
	ans.max.setY(std::max(a.max.getY(), b.max.getY()));
	return ans;
}

float AABB::GetPerimeter() const
{
	float width = max.getX() - min.getY();
	float height = max.getY() - min.getY();

	return 2.0f*(width+height);
}

bool AABB::Overlap(const AABB& other) const
{
	if (max.getX() < other.min.getX() || other.max.getX() < min.getX()) return false;
	if (max.getY() < other.min.getY() || other.max.getY() < min.getY()) return false;
	return true;
}

bool AABB::Contains(const AABB& other) const
{
	return other.min.getX() >= min.getX() && other.min.getY() >= min.getY() &&
		other.max.getX() <= max.getX() && other.max.getY() <= max.getY();
}

float AABB::RayCast(const RayCastInput& input) const {
    float tmin = -FLT_MAX;
    float tmax = FLT_MAX;

    Vector2 p = input.p1;
    Vector2 d = input.p2 - input.p1; // 射线方向矢量

    // 针对 X 轴和 Y 轴分别计算
    for (int i = 0; i < 2; ++i) {
        float absD = (i == 0) ? std::abs(d.getX()) : std::abs(d.getY());
        float dVal = (i == 0) ? d.getX() : d.getY();
        float minVal = (i == 0) ? min.getX() : min.getY();
        float maxVal = (i == 0) ? max.getX() : max.getY();
        float pVal = (i == 0) ? p.getX() : p.getY();

        if (absD < FLT_EPSILON) {
            // 射线与该轴平行，如果起点不在范围内，则永不相交
            if (pVal < minVal || pVal > maxVal) return -1.0f;
        }
        else {
            // 计算进入和离开 AABB 平面的时间 t
            float t1 = (minVal - pVal) / dVal;
            float t2 = (maxVal - pVal) / dVal;

            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);

            if (tmin > tmax) return -1.0f; // 不相交
        }
    }

    if (tmin >= 0.0f && tmin <= input.maxFraction) {
        return tmin; // 返回进入时间
    }

    return -1.0f;
}
