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