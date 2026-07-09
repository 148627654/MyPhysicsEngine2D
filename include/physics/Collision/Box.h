#pragma once // 记得加上这个，防止重复包含
#include "Shape.h"
class Box : public Shape
{
public:
	Box(float w = 0.0,float h=0.0) :width(w), height(h)
	{
		type = type_Box;
	}

	float getArea() { return width * height; }
	MassData ComputeMass(float density);
private:
	float width;
	float height;
};