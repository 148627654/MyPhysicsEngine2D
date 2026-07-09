#pragma once // 记得加上这个，防止重复包含
#include "Shape.h"
#include "../Common/Setting.h"
class Circle : public Shape
{
public:
	Circle(float r = 0.0) :radius(r)
	{
		type = type_Circle;
	}
	float getArea() { return Settings::PAI * radius * radius; }
	MassData ComputeMass(float density);
private:
	float radius;
};