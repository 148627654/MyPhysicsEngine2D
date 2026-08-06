#pragma once // 记得加上这个，防止重复包含
#include "../include/physics/Collision/AABB.h"
struct MassData {
	float mass;    // 质量
	float inertia; // 转动惯量
};

struct RayCastOutput {
	Vector2 normal;   // 撞击点的表面法线
	float fraction;   // 撞击点在射线上的比例 [0, 1]
};

struct AABB;
class Shape
{
public:
	
	enum Type { type_Circle, type_Box };
	Type type;
	virtual MassData ComputeMass(float density) = 0;//
	virtual float getArea() = 0;
	virtual ~Shape() {}
	virtual AABB ComputeAABB(Vector2 pos , float angle) = 0;
	virtual bool RayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation) = 0;
	// 获取形状的扫掠半径（中心到最远点的距离）
	virtual float GetSweepRadius() const = 0;
};