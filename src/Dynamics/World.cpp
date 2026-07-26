#include "World.h"
#include "../Collision/Collision.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"
void World::Step(float dt) {
    // --- 1. 速度积分 (Velocity Integration) ---
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f) continue;

        // v = v + (f/m + g) * dt
        Vector2 acceleration = b->force * b->getInvMass() + m_gravity * b->gravityScale;
        b->velocity += acceleration * dt;

        // w = w + (torque/I) * dt
        float angularAcc = b->torque * b->getInvInertia();
        b->angularVelocity += angularAcc * dt;

        b->ClearForce(); // 每帧清空力累加器
        b->torque = 0.0f;
    }

    // --- 2. 位置积分 与 宽相维护 (Position Integration & Sync) ---
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f) continue;

        // 更新位置和角度 (内部会自动触发 b->updateAABB())
        b->SetPosition(b->GetPosition() + b->velocity * dt);
        b->SetRotation(b->GetRotation() + b->angularVelocity * dt);

        // 【关键集成】：同步到宽相
        // 只有当物体跳出肥包围盒时，MoveProxy 会返回 true 并记录到 MoveBuffer
        m_broadPhase.MoveProxy(b->getProxyId(), b->GetAABB(), b->velocity * dt);
    }

    // --- 3. 碰撞对收集 (Narrow Phase Dispatch) ---
    m_manifolds.clear();

    // 定义回调逻辑
    auto broadPhaseCallback = [&](void* userDataA, void* userDataB) {
        Body* bodyA = static_cast<Body*>(userDataA);
        Body* bodyB = static_cast<Body*>(userDataB);

        // 两个静态物体不解算
        if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) return;

        // 执行窄相精确检测 (Dispatch)
        Manifold m(bodyA, bodyB);
        // 这里是你 V1 阶段实现的碰撞分发器逻辑
        if (Collision::Dispatch(&m, bodyA, bodyB)) {
            m_manifolds.push_back(m);
        }
        printf("[COLLISION] Body A and B overlapped!\n");
        };

    // 【核心提升】：宽相只对“动过”的物体执行查询，极大减少检测次数
    m_broadPhase.UpdatePairs(broadPhaseCallback);

    // --- 4. 冲量解算 (Solving) ---
    const int velocityIterations = 8;
    for (int i = 0; i < velocityIterations; ++i) {
        for (auto& manifold : m_manifolds) {
            ImpulseSolver(manifold); // 你的冲量解算器
        }
    }

    // --- 5. 位置修正 (Baumgarte) ---
    for (auto& manifold : m_manifolds) {
        PositionalCorrection(manifold);
    }
}

void World::RemoveBody(Body* body) {
	// 1. 从宽相中安全摘除
	if (body->getProxyId() != -1) {
		m_broadPhase.DestroyProxy(body->getProxyId());
	}

	// 2. 从列表移除并释放内存
	auto it = std::find(m_bodies.begin(), m_bodies.end(), body);
	if (it != m_bodies.end()) {
		m_bodies.erase(it);
		delete body;
	}
}

void World::RayCast(Vector2 p1, Vector2 p2) {
	RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1.0f;

	// 局部变量：用来追踪查询过程中的最近距离
	// 虽然函数不返回结果，但我们在内部可以通过它来优化树的搜索
	float closestFraction = 1.0f;

	// 定义宽相回调：树发现射线经过了某个 AABB
	auto broadPhaseCallback = [&](RayCastInput& subInput, int32_t proxyId) -> float {
		// 1. 获取 Body
		void* userData = m_broadPhase.GetUserData(proxyId);
		Body* body = static_cast<Body*>(userData);

		// 2. 执行真正的窄相检测 (Box 或 Circle 的 RayCast)
		RayCastOutput output;
		bool hit = body->GetShape()->RayCast(&output, subInput, body->GetPosition(), body->GetRotation());

		if (hit) {
			// 3. 既然撞到了真实的形状，我们打印详细信息
			printf("[RAY HIT] Body at (%f, %f)\n", body->GetPosition().getX(), body->GetPosition().getY());
			printf("          Fraction: %f\n", output.fraction);
			printf("          Normal: (%f, %f)\n", output.normal.getX(), output.normal.getY());

			// 4. 更新最近距离
			closestFraction = output.fraction;

			// 5. 返回当前撞击的比例。
			// 这是一个巨大的优化：树接收到这个值后，会自动“剪枝”，
			// 之后它只会去检查比这个点更近的 AABB 分支。
			return output.fraction;
		}

		// 没撞到具体的形状（只是经过了肥包围盒的边缘），继续寻找下一个
		return 1.0f;
		};

	// 6. 调用宽相进行全局扫描
	m_broadPhase.RayCast(input, broadPhaseCallback);
}