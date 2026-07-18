#pragma once // 记得加上这个，防止重复包含

#include "../Dynamics/Body.h"
#include "../Common/Vector2.h"
#include <vector>
struct Manifold
{
public:
	Manifold(Body* a, Body* b)
		:bodyA(a), bodyB(b), normal(Vector2(0, 0)), penetration(0) {
		impulseN[ 0 ] = impulseN[ 1 ] = 0.0f;
		impulseT[ 0 ] = impulseT[ 1 ] = 0.0f;
	}
	Body* bodyA;					//参与碰撞的第一个物体
	Body* bodyB;					//参与碰撞的第二个物体
	Vector2 normal;					//碰撞法线（由 A 指向 B）
	float penetration;				//穿透深度
	//12 天做出了修改，如果只是矩形的情况下只有两个碰撞点，可以减少堆区的时间浪费
	Vector2 contacts[2];	//接触点列表（圆圆碰撞通常只有一个点）。
	int contactCount;     // 实际有效的接触点数量 (1 或 2)
	// --- 关键：为了第 12 天的稳定解算，我们需要记录上一次施加的冲量 ---
	float impulseN[ 2 ];    // 每个接触点的累积法向冲量
	float impulseT[ 2 ];    // 每个接触点的累积切向（摩擦）冲量
};