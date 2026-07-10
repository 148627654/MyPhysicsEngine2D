#pragma once // 记得加上这个，防止重复包含

#include "../Dynamics/Body.h"
#include "../Common/Vector2.h"
#include <vector>
struct Manifold
{
public:
	Manifold(Body* a, Body* b)
		:bodyA(a), bodyB(b), normal(Vector2(0, 0)), penetration(0) {}
	Body* bodyA;					//参与碰撞的第一个物体
	Body* bodyB;					//参与碰撞的第二个物体
	Vector2 normal;					//碰撞法线（由 A 指向 B）
	float penetration;				//穿透深度
	std::vector<Vector2> contacts;	//接触点列表（圆圆碰撞通常只有一个点）。
};