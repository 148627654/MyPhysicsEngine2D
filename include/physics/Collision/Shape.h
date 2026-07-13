#pragma once // 记得加上这个，防止重复包含
#include "../include/physics/Collision/AABB.h"
struct MassData {
	float mass;    // 质量
	float inertia; // 转动惯量
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
	
};