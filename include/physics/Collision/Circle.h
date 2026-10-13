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
	AABB computeAABB(Vector2 pos , float angle) override;
	float getArea() const override { return Settings::PAI * radius * radius; }
	MassData computeMass(float density) override;
	float getR()const { return radius; }
	bool rayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation);
	// 点包含判定：点到圆心的距离平方 ≤ R²
	bool TestPoint(const Vector2& localPoint) const;
	float getSweepRadius() const { return getR(); }
private:
	float radius;
};