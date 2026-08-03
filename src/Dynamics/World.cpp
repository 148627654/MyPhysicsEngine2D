#include "World.h"
#include "../Collision/Collision.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"


void World::Step(float dt) {
    // --- 1. 同步坐标 (这一步必须最先做) ---
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f) continue;

        if (!b->IsAwake()) {
            // 【补充：状态断言/强制归零】
            // 如果物体是睡着的，它的速度必须是 0。
            assert(b->GetVelocity().LengthSquared() == 0); 
            // 或者保险起见直接强制设为 0。
            b->velocity.Clear();
            b->angularVelocity = 0.0f;
            continue;
        }

        b->updateAABB();
        m_broadPhase.MoveProxy(b->getProxyId(), b->GetAABB(), b->GetVelocity() * dt);
    }

    // --- 2. 存活检查 (Persistence) ---
    // 只要 AABB 还重叠，Contact 就得活着！
    for (auto it = m_contactMap.begin(); it != m_contactMap.end(); ) {
        Contact* c = it->second;

        // 主动询问宽相：Fat AABB 是否还重合？
        if (m_broadPhase.TestOverlap(c->m_bodyA->getProxyId(), c->m_bodyB->getProxyId())) {
            c->update(); // 执行窄相，刷新 IsTouching 状态
            it++;
        }
        else {
            // 只有宽相说分开了，才真正销毁
            RemoveContactFromGraph(c);
            delete c;
            it = m_contactMap.erase(it);
        }
    }

    // --- 3. 发现新碰撞 ---
    m_broadPhase.UpdatePairs([&](void* uA, void* uB) {
        Body* bodyA = static_cast<Body*>(uA);
        Body* bodyB = static_cast<Body*>(uB);
        if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) return;
        if (bodyA > bodyB) std::swap(bodyA, bodyB);
        auto key = std::make_pair(bodyA, bodyB);

        if (m_contactMap.count(key)) return;

        Contact* contact = new Contact(bodyA, bodyB);
        m_contactMap[key] = contact;
        AddContactToGraph(contact);
        contact->update();
        });

    // --- 4. 构建岛屿并解算 (核心) ---
    // DFS 内部会重置它自己的 m_islandFlag，不需要在这里操心
    BuildAndSolveIslands(dt);
}

void World::AddContactToGraph(Contact* c) {
    Body* bodyA = c->m_bodyA;
    Body* bodyB = c->m_bodyB;

    c->m_nodeA.next = bodyA->m_contactList;
    if (bodyA->m_contactList) bodyA->m_contactList->prev = &c->m_nodeA;
    bodyA->m_contactList = &c->m_nodeA;

    c->m_nodeB.next = bodyB->m_contactList;
    if (bodyB->m_contactList) bodyB->m_contactList->prev = &c->m_nodeB;
    bodyB->m_contactList = &c->m_nodeB;
}

void World::RemoveContactFromGraph(Contact* c) {
    // Body A 侧
    if (c->m_nodeA.prev) c->m_nodeA.prev->next = c->m_nodeA.next;
    if (c->m_nodeA.next) c->m_nodeA.next->prev = c->m_nodeA.prev;
    if (c->m_bodyA->m_contactList == &c->m_nodeA) c->m_bodyA->m_contactList = c->m_nodeA.next;

    // Body B 侧
    if (c->m_nodeB.prev) c->m_nodeB.prev->next = c->m_nodeB.next;
    if (c->m_nodeB.next) c->m_nodeB.next->prev = c->m_nodeB.prev;
    if (c->m_bodyB->m_contactList == &c->m_nodeB) c->m_bodyB->m_contactList = c->m_nodeB.next;
}

void World::RemoveBody(Body* body) {
    if (body == nullptr) return;

    // 1. 【核心修复】：从碰撞图中彻底抹除该物体
    ContactEdge* ce = body->getContactList();
    while (ce != nullptr) {
        Contact* contact = ce->contact;
        ce = ce->next; // 提前保存下一个，因为当前的 contact 马上要被删了

        // A. 唤醒邻居（防止悬空）
        contact->m_bodyA->setAwake(true);
        contact->m_bodyB->setAwake(true);

        // B. 从全局 map 中移除（根据 A,B 指针生成的 key）
        Body* bA = contact->m_bodyA;
        Body* bB = contact->m_bodyB;
        if (bA > bB) std::swap(bA, bB);
        auto key = std::make_pair(bA, bB);
        m_contactMap.erase(key);

        // C. 从两个物体的双向链表中安全摘除（调用你之前写的辅助函数）
        RemoveContactFromGraph(contact);

        // D. 销毁 Contact 对象
        delete contact;
    }

    // 2. 从宽相树中移除代理
    if (body->getProxyId() != -1) {
        m_broadPhase.DestroyProxy(body->getProxyId());
    }

    // 3. 从世界物体列表中移除
    auto it = std::find(m_bodies.begin(), m_bodies.end(), body);
    if (it != m_bodies.end()) {
        m_bodies.erase(it);
    }

    // 4. 最后才真正释放内存
    delete body;
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
        //检查该岛屿是否符合休眠条件。
        if (seed->m_islandFlag || seed->getInvMass() == 0.0f||!seed->IsAwake()) continue;

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
                if (other->getInvMass() > 0.0f) {
                    if (other->m_islandFlag) continue; // 已经处理过的动态物体，跳过

                    if (!other->IsAwake()) {
                        other->setAwake(true);
                    }

                    // 标记并入栈，继续向外传染岛屿
                    other->m_islandFlag = true;
                    stack.push_back(other);
                }
                
                else {
                    island.Add(other);
                }
            }
        }
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