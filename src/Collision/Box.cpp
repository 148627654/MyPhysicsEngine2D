#include "Box.h"

MassData Box::ComputeMass(float density)
{
    MassData data;
    data.mass = density * getArea();
    data.inertia = data.mass / 12 * (width * width + height * height);
    return data;
}
