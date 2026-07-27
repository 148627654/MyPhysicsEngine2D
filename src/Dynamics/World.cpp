#include "World.h"
#include "../Collision/Collision.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"
// 在 World.h 中添加：std::map<std::pair<Body*, Body*>, Contact*> m_contactMap;

void World::Step(float dt) {
    // --- 1. 宽相发现 & 窄相更新 (持久化 Contact) ---
    // 我们需要一个标记位来识别哪些 Contact 在这一帧消失了
    for (auto& pair : m_contactMap) {
        pair.second->m_islandFlag = false; // 临时借用这个标记做“存活检查”
    }

    auto broadPhaseCallback = [&](void* userDataA, void* userDataB) {
        Body* bodyA = static_cast<Body*>(userDataA);
        Body* bodyB = static_cast<Body*>(userDataB);

        if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) return;

        // 保证顺序 A < B
        if (bodyA > bodyB) std::swap(bodyA, bodyB);
        auto key = std::make_pair(bodyA, bodyB);

        Contact* contact = nullptr;
        if (m_contactMap.count(key)) {
            contact = m_contactMap[key];
        }
        else {
            // 创建新碰撞并挂载双向链表
            contact = new Contact(bodyA, bodyB);
            m_contactMap[key] = contact;

            // 挂载到 Body A
            contact->m_nodeA.next = bodyA->m_contactList;
            if (bodyA->m_contactList) bodyA->m_contactList->prev = &contact->m_nodeA;
            bodyA->m_contactList = &contact->m_nodeA;

            // 挂载到 Body B
            contact->m_nodeB.next = bodyB->m_contactList;
            if (bodyB->m_contactList) bodyB->m_contactList->prev = &contact->m_nodeB;
            bodyB->m_contactList = &contact->m_nodeB;
        }

        contact->update(); // 执行窄相检测
        if (contact->IsTouching()) {
            contact->m_islandFlag = true; // 标记本帧活跃
        }
        };

    m_broadPhase.UpdatePairs(broadPhaseCallback);

    // --- 2. 构建并解算岛屿 ---
    BuildAndSolveIslands(dt);

    // --- 3. 清理不再接触的 Contact (Mark and Sweep) ---
    for (auto it = m_contactMap.begin(); it != m_contactMap.end(); ) {
        Contact* c = it->second;
        // 如果 AABB 不重合了或者 IsTouching 没了，且标记为 false
        if (!c->m_islandFlag) {
            // 从 Body A 链表摘除
            if (c->m_nodeA.prev) c->m_nodeA.prev->next = c->m_nodeA.next;
            if (c->m_nodeA.next) c->m_nodeA.next->prev = c->m_nodeA.prev;
            if (c->m_bodyA->m_contactList == &c->m_nodeA) c->m_bodyA->m_contactList = c->m_nodeA.next;

            // 从 Body B 链表摘除
            if (c->m_nodeB.prev) c->m_nodeB.prev->next = c->m_nodeB.next;
            if (c->m_nodeB.next) c->m_nodeB.next->prev = c->m_nodeB.prev;
            if (c->m_bodyB->m_contactList == &c->m_nodeB) c->m_bodyB->m_contactList = c->m_nodeB.next;

            delete c;
            it = m_contactMap.erase(it);
        }
        else {
            ++it;
        }
    }

    // --- 4. 同步 AABB ---
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f) continue;
        b->updateAABB();
        m_broadPhase.MoveProxy(b->getProxyId(), b->GetAABB(), b->GetVelocity() * dt);
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

void World::BuildAndSolveIslands(float dt) {
    // 1. 初始化标记
    for (Body* b : m_bodies) b->m_islandFlag = false;
    for (auto& pair : m_contactMap) pair.second->m_islandFlag = false;

    TimeStep step;
    step.dt = dt;
    step.velocityIterations = 8;
    step.positionIterations = 3;

    int islandCount = 0; // 诊断：记录本帧生成的岛屿总数

    // 2. 遍历所有物体寻找“种子”
    for (Body* seed : m_bodies) {
        if (seed->m_islandFlag || seed->getInvMass() == 0.0f) continue;

        // --- DFS 诊断日志：发现新岛屿 ---
        islandCount++;

        IsLand island(m_bodies.size(), m_contactMap.size());

        // 3. DFS 遍历
        std::vector<Body*> stack;
        stack.reserve(m_bodies.size());
        stack.push_back(seed);
        seed->m_islandFlag = true;

        int bodiesInIsland = 0;
        int contactsInIsland = 0;

        while (!stack.empty()) {
            Body* b = stack.back();
            stack.pop_back();

            island.Add(b);
            bodiesInIsland++; // 计数

            for (ContactEdge* ce = b->getContactList(); ce; ce = ce->next) {
                Contact* contact = ce->contact;

                if (contact->m_islandFlag || !contact->IsTouching()) continue;

                island.Add(contact);
                contact->m_islandFlag = true;
                contactsInIsland++; // 计数

                Body* other = ce->other;
                if (other->m_islandFlag) continue;

                if (other->getInvMass() > 0.0f) {
                    other->m_islandFlag = true;
                    stack.push_back(other);
                }
                else {
                    island.Add(other);
                    // 静态物体不标记，bodiesInIsland 不统计它（可选，看你需求）
                }
            }
        }

        // --- DFS 诊断日志：岛屿构建完成 ---
        // 仅在每一秒（约60帧）打印一次，或者在物体数量变动时打印，防止刷屏
        // 这里为了测试先直接打印
        
        //Logger::Info("DFS: Island " + std::to_string(islandCount) +
        //             " created with " + std::to_string(bodiesInIsland) +
        //             " dynamic bodies and " + std::to_string(contactsInIsland) + " contacts.");
        //

        // 4. 解算
        island.Solve(step, m_gravity);
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