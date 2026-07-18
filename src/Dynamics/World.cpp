#include "World.h"
#include "../Collision/Collision.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"
void World::Step(float dt)
{
	// --- 1. 力与速度积分 (Linear & Angular Velocity) ---
	for (auto p : m_bodies)
	{
		if (p->getInvMass() == 0) continue; // 静态物体跳过

		// 计算加速度并更新速度 v = v + a*dt
		Vector2 acc = p->force * p->getInvMass() + m_gravity * p->gravityScale;
		p->velocity += acc * dt;

		// 计算角加速度并更新角速度 w = w + alpha*dt
		float angularAcc = p->torque * p->getInvInertia();
		p->angularVelocity += angularAcc * dt;

		// 清空力矩和力，为下一帧做准备
		p->ClearForce();
		p->torque = 0.0f;
	}

	// --- 2. 碰撞检测与流形生成 ---
	m_manifolds.clear();
	for (size_t i = 0; i < m_bodies.size(); ++i) {
		for (size_t j = i + 1; j < m_bodies.size(); ++j) {
			Body* a = m_bodies[i];
			Body* b = m_bodies[j];

			// 两个静态物体不碰撞
			if (a->getInvMass() == 0 && b->getInvMass() == 0) continue;

			// 宽相过滤
			if (!Collision::AABBvsAABB(a->GetAABB(), b->GetAABB())) continue;

			// 窄相检测
			Manifold m(a, b);
			if (Collision::Dispatch(&m, a, b)) {
				m_manifolds.push_back(m);
			}
		}
	}

	// --- 3. 速度解算阶段 (Sequential Impulses) ---
	// 【第 12 天核心】：通过多次迭代让系统趋于稳定
	const int velocityIterations = 8; // 建议 8-10 次
	for (int i = 0; i < velocityIterations; ++i) {
		for (auto& m : m_manifolds) {
			// 这个函数现在包含：多点处理、法向冲量、摩擦力冲量
			ImpulseSolver(m);
		}
	}

	// --- 4. 位置积分 (Position Integration) ---
	// 使用解算后的速度更新位置
	for (auto p : m_bodies) {
		if (p->getInvMass() == 0) continue;

		p->position += p->velocity * dt;
		p->rotation += p->angularVelocity * dt;

		// 更新 AABB 以供下一帧宽相检测使用
		p->updateAABB();
	}

	// --- 5. 位置修正 (Position Correction / Baumgarte Stabilization) ---
	// 解决物体相互穿透（下陷）的问题
	for (auto& m : m_manifolds) {
		PositionalCorrection(m);
	}
}