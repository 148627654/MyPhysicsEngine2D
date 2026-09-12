#pragma once // 记得加上这个，防止重复包含
#include "Shape.h"
#include "../Common/Setting.h"

class Circle : public Shape
{
public:
	Circle(float r = 0.0f, const Physics2D::Material& mat = Physics2D::Material())
		: Shape(Type::type_Circle, mat), radius(r)
	{
	}
	AABB ComputeAABB(Vector2 pos , float angle) override;
	float getArea() const override { return Settings::PAI * radius * radius; }
	MassData ComputeMass(float density) override;
	float getR()const { return radius; }
	bool RayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation);
	float GetSweepRadius() const { return getR(); }
private:
	float radius;
};