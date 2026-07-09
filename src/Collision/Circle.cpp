#include "Circle.h"


MassData Circle::ComputeMass(float density)
{
    MassData data;
    data.mass= density * getArea();
    data.inertia = data.mass / 2 * radius * radius;
    return data;
}
