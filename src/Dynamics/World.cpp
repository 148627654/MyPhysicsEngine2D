#include "World.h"
#include "../Collision/Collision.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"
void World::Step(float dt)
{
	// --- 1. 力与速度积分 (Velocity Integration) ---
	for (auto p : m_bodies)
	{
		if (p->getInvMass() == 0) continue;

		Vector2 acc = p->force * p->getInvMass() + m_gravity * p->gravityScale;
		p->velocity += acc * dt;

		float angularAcc = p->torque * p->getInvInertia();
		p->angularVelocity += angularAcc * dt;

		p->ClearForce();
		p->torque = 0.0f;
	}

	// --- 2. 位置积分与宽相更新  ---
	// 逻辑：在检测碰撞前，必须先更新物体的坐标和 AABB，并通知宽相树
	for (auto p : m_bodies) {
		if (p->getInvMass() == 0) continue;

		p->position += p->velocity * dt;
		p->rotation += p->angularVelocity * dt;

		// 更新物体的 Tight AABB
		p->updateAABB();

		// 【关键】：通知宽相系统，物体移动了
		// 如果物体超出了它的“肥包围盒”，MoveProxy 会把它的 ID 放入移动缓冲区
		m_broadPhase.MoveProxy(p->getProxyId(), p->GetAABB(), p->velocity * dt);
	}

	// --- 3. 碰撞检测 (BroadPhase + NarrowPhase) ---
	m_manifolds.clear();

	// 定义回调函数：当宽相发现一对潜在碰撞时，执行窄相检测
	auto broadPhaseCallback = [&](void* userDataA, void* userDataB) {
		Body* a = static_cast<Body*>(userDataA);
		Body* b = static_cast<Body*>(userDataB);

		// 两个静态物体不碰撞 (双重保险)
		if (a->getInvMass() == 0 && b->getInvMass() == 0) return;

		// 执行窄相检测（Dispatch 内部会处理 Box vs Box 等具体算法）
		Manifold m(a, b);
		if (Collision::Dispatch(&m, a, b))
			m_manifolds.push_back(m);
		};
	m_broadPhase.UpdatePairs(broadPhaseCallback);

	// --- 4. 速度解算阶段 (Sequential Impulses) ---
	const int velocityIterations = 8;
	for (int i = 0; i < velocityIterations; ++i) {
		for (auto& m : m_manifolds) {
			ImpulseSolver(m);
		}
	}

	// --- 5. 位置修正 (Baumgarte Stabilization) ---
	for (auto& m : m_manifolds) {
		PositionalCorrection(m);
	}
}