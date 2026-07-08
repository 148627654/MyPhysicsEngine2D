#include "World.h"

void World::Step(float dt)
{
	//遍历每一个实体
	for (auto p : m_bodies)
	{
		if (p->invMass == 0)
			continue;
		//计算加速度
		Vector2 acc = (p->force) * (p->invMass);
		//重力产生的加速度
		acc += m_gravity * p->gravityScale;
		//执行**半隐式欧拉积分更新速度和位置。

		p->velocity += acc * dt;
		p->position += p->velocity * dt;

		// 清力
		p->ClearForce();
	}
}
