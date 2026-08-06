#include "World.h"
#include "../Collision/Collision.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"
#include "../Collision/TimeOfImpact.h"

void World::Step(float dt) {
    // --- 1. 备份与预测积分 ---
    for (Body* b : m_bodies) {
        b->SavePrevState(); // 记录 t0 (起始位置)

        if (b->getInvMass() > 0.0f && b->IsAwake()) {
            // A. 速度积分 (v = v + a*dt)
            Vector2 accel = b->getForce() * b->getInvMass() + m_gravity * b->getGravityScale();
            b->SetVelocity(b->GetVelocity() + accel * dt);

            // B. 位置预测 (P1 = P0 + v*dt) -> 此时位置变成了本帧结束时的位置
            b->SetPosition(b->GetPosition() + b->GetVelocity() * dt);
            b->SetRotation(b->GetRotation() + b->getAngularVelocity() * dt);

            b->ClearForce();
            b->setTorque(0.0f);
        }
    }

    // --- 2. 宽相更新 (基于预测后的 P1 位置) ---
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f || !b->IsAwake()) continue;

        b->updateAABB(); // 必须基于 P1 刷新 AABB
        // 对于子弹，使用扫掠 AABB (涵盖 P0 到 P1 的轨迹)
        AABB bpAABB = b->IsBullet() ? b->GetSweptAABB(dt) : b->GetAABB();
        m_broadPhase.MoveProxy(b->getProxyId(), bpAABB, b->GetVelocity() * dt);
    }

    // --- 3. 碰撞维护与 TOI 计算 ---
    // A. 存活检查
    for (auto it = m_contactMap.begin(); it != m_contactMap.end(); ) {
        Contact* c = it->second;
        if (m_broadPhase.TestOverlap(c->m_bodyA->getProxyId(), c->m_bodyB->getProxyId())) {
            c->update();    // 离散窄相检测
            UpdateTOI(c, dt); // 计算 [P0, P1] 轨迹上的撞击时间
            it++;
        }
        else {
            RemoveContactFromGraph(c);
            delete c;
            it = m_contactMap.erase(it);
        }
    }

    // B. 发现新碰撞
    m_broadPhase.UpdatePairs([&](void* uA, void* uB) {
        Body* bodyA = (Body*)uA;
        Body* bodyB = (Body*)uB;
        if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) return;
        if (bodyA > bodyB) std::swap(bodyA, bodyB);
        auto key = std::make_pair(bodyA, bodyB);

        if (m_contactMap.count(key)) return;

        Contact* contact = new Contact(bodyA, bodyB);
        m_contactMap[key] = contact;
        AddContactToGraph(contact);
        contact->update();
        UpdateTOI(contact, dt);
        });

    // --- 4. CCD 拦截与回溯 (核心拦截) ---
    // 我们需要找出这一帧中所有发生穿墙预兆的物体
    for (auto const& [key, contact] : m_contactMap) {
        // 如果 TOI 显著小于 1.0，说明在本帧结束前就会撞上
        // 增加 0.001 的偏移防止由于浮点误差导致的“粘墙”现象
        if (contact->m_toi < 0.999f && contact->m_toi > 0.001f) {

            // 找到撞击瞬间的位姿 (插值)
            Transform tfA = contact->m_bodyA->GetTransform(contact->m_toi, dt);
            Transform tfB = contact->m_bodyB->GetTransform(contact->m_toi, dt);

            // 拦截：强制把物体从“穿透点”拉回到“撞击点”
            contact->m_bodyA->SetTransform(tfA);
            contact->m_bodyB->SetTransform(tfB);

            // 关键：位置变了，必须重新执行窄相，生成撞击点的流形 (Manifold)
            // 这样接下来的 Solve 环节才能根据此时的法线计算反弹冲量
            contact->update();

            // 诊断日志
            Logger::Info(">>> [CCD] Alpha " + std::to_string(contact->m_toi) +
                " Intercepted at X: " + std::to_string(tfA.p.getX()));
        }
    }

    // --- 5. 岛屿解算 ---
    // 此时物体的状态：
    // - 没撞上的：处于预测的 P1 位置。
    // - 撞上的：被回退到了撞击点位置。
    // BuildAndSolveIslands 内部的 Solve 必须已经去掉了再次积分的逻辑！
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

void World::WakeNeighbors(Body* body)
{
    if (body == nullptr || body->getInvMass() == 0.0f) return;
    ContactEdge* ce = body->getContactList();
    while (ce != nullptr) {
        Body* other = ce->other;

        // 关键：只唤醒动态物体（静态物体不需要醒）
        if (other->getInvMass() > 0.0f) {
            other->setAwake(true); // 这个函数内部会重置 timer
        }

        ce = ce->next;
    }
}

void World::UpdateTOI(Contact* c, float dt) {
    if (!c->m_bodyA->IsBullet() && !c->m_bodyB->IsBullet()) return;
    
    TOIInput input;
    input.bodyA = c->m_bodyA;
    input.bodyB = c->m_bodyB;
    input.dt = dt;
    input.tolerance = Settings::LINEAR_SLOP;

    TOIOutput output = TimeOfImpact::Solve(input);

    // 【核心修正】：只要判定为 Hit 或 Overlapped，都要记录 alpha
    if (output.state == TOIOutput::Hit || output.state == TOIOutput::Overlapped) {
        c->m_toi = output.alpha;
        Logger::Info(">>> [TOI SUCCESS] Alpha: " + std::to_string(c->m_toi));
    }
    else {
        // --- 增加这行诊断日志 ---
        //Logger::Info(">>> [TOI FAILED] State: " + std::to_string(output.state));
        c->m_toi = 1.0f;
    }
}