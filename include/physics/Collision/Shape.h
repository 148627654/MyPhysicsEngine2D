#pragma once // 记得加上这个，防止重复包含

struct MassData {
	float mass;    // 质量
	float inertia; // 转动惯量
};

class Shape
{
public:
	enum Type { type_Circle, type_Box };
	Type type;
	virtual MassData ComputeMass(float density) = 0;//
	virtual float getArea() = 0;
	virtual ~Shape() {}

	
};