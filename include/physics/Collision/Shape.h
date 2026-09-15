#pragma once // 记得加上这个，防止重复包含
#include "AABB.h"
#include "../Dynamics/Material.h"
struct MassData {
	float mass;    // 质量
	float inertia; // 转动惯量
};

struct RayCastOutput {
	Vector2 normal;   // 撞击点的表面法线
	float fraction;   // 撞击点在射线上的比例 [0, 1]
};

class Shape
{
public:

	enum Type { type_Circle, type_Box, type_Capsule,type_Polygon};
	Type type;
	virtual MassData ComputeMass(float density) = 0;//
	virtual float getArea() const = 0;
	Shape(Type type, const Physics2D::Material& mat = Physics2D::Material())
		: type(type), material(mat), isTrigger(false) {
	}
	virtual ~Shape() {}
	virtual AABB ComputeAABB(Vector2 pos , float angle) = 0;
	virtual bool RayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation) = 0;
	// 获取形状的扫掠半径（中心到最远点的距离）
	virtual float GetSweepRadius() const = 0;

	Physics2D::Material material; // 物理材质属性（密度/摩擦/恢复系数），唯一数据源在 Shape
	bool isTrigger = false; // 是否为触发器（只产生接触信息，不做物理响应）
};