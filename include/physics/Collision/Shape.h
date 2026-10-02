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

/// @brief 形状基类（Box2D 中 Fixture 的角色）
///
/// 职责：
///  - 几何描述：包围盒（computeAABB）、射线检测（rayCast）、扫掠半径（getSweepRadius）
///  - 物理属性：材质（material：密度/摩擦/恢复系数）与触发器标记（isTrigger）
///  - 质量计算：由密度推导质量与转动惯量（computeMass）
///
/// 子类：Circle、Box、Capsule、Polygon（凸多边形）
class Shape
{
public:

	enum Type { type_Circle, type_Box, type_Capsule,type_Polygon};
	Type type;
	virtual MassData computeMass(float density) = 0;//
	virtual float getArea() const = 0;
	Shape(Type type, const Physics2D::Material& mat = Physics2D::Material())
		: type(type), material(mat), isTrigger(false) {
	}
	virtual ~Shape() {}
	virtual AABB computeAABB(Vector2 pos , float angle) = 0;
	virtual bool rayCast(RayCastOutput* output, RayCastInput& input,
		const Vector2& position, float rotation) = 0;
	// 获取形状的扫掠半径（中心到最远点的距离）
	virtual float getSweepRadius() const = 0;

	Physics2D::Material material; // 物理材质属性（密度/摩擦/恢复系数），唯一数据源在 Shape
	bool isTrigger = false; // 是否为触发器（只产生接触信息，不做物理响应）
};