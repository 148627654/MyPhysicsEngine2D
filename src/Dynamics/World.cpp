#include "World.h"
#include "../Collision/Collision.h"

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
		float angularAcceleration = p->torque * p->invInertia;
		// 2. 积分角速度
		p->angularVelocity += angularAcceleration * dt;
		// 3. 积分角度
		p->rotation += p->angularVelocity * dt;
		// 4. 清空转矩
		p->torque = 0.0f;
	}

	for (auto& body : m_bodies) {
		body->updateAABB();
	}

	m_manifolds.clear();
	// 4. 碰撞检测循环 (宽相 + 窄相)
	for (size_t i = 0; i < m_bodies.size(); ++i) {
		for (size_t j = i + 1; j < m_bodies.size(); ++j) {
			Body* a = m_bodies[i];
			Body* b = m_bodies[j];

			// 宽相过滤 (Day 07 核心)
			if (!Collision::AABBvsAABB(a->GetAABB(), b->GetAABB())) {
				continue;
			}

			// 窄相检测
			Manifold m(a, b);
			if (Collision::Dispatch(&m, a, b)) {
				// 如果撞了，把这个流形存入列表
				m_manifolds.push_back(m);
			}
		}
	}
}
