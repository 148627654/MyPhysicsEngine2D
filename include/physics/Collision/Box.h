#pragma once // 记得加上这个，防止重复包含
#include "Shape.h"
#include <vector>
class Box : public Shape
{
public:
	Box(float w = 0.0,float h=0.0) :width(w), height(h)
	{
		type = type_Box;
	}
	AABB ComputeAABB(Vector2 pos , float angle);
	float getArea() { return width * height; }
	MassData ComputeMass(float density);
	bool RayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation);

	//获取长和宽
	float getWidth()const { return width; }
	float getHeigh()const { return height; }
	float getHalfWidth()const { return width * 0.5f; }
	float getHalfHeigh()const { return height * 0.5f; }
	// 获取形状的扫掠半径（中心到最远点的距离）
	float GetSweepRadius() const;
private:
	float width;
	float height;
};