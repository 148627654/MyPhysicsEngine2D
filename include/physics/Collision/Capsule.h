#pragma once
#include "Shape.h"
#include "../Common/Vector2.h"
#include "../Common/Setting.h"
class Capsule : public Shape
{
public:
	Capsule(float radius, float length, const Physics2D::Material& mat = Physics2D::Material())
		: Shape(Shape::type_Capsule, mat), m_radius(radius), m_length(length) {
	}
	~Capsule() {}

	MassData ComputeMass(float density)override;//
	float getArea() const override;
	bool RayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation)override;
	AABB ComputeAABB(Vector2 pos, float angle);
	float GetSweepRadius() const;
	// 获取局部骨架端点 (默认沿 Y 轴分布)
	Vector2 GetLocalSegmentA() const { return Vector2(0.0f, -m_length * 0.5f); }
	Vector2 GetLocalSegmentB() const { return Vector2(0.0f, m_length * 0.5f); }

	inline float GetRadius() const { return m_radius; }
	inline float GetLength() const { return m_length; }
private:
	float m_radius; // 半径
	float m_length; // 中间直线段长度
};