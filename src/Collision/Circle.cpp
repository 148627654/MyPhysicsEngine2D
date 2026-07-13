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
