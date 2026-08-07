#include "World.h"
#include "../Collision/Collision.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"
#include "../Collision/TimeOfImpact.h"
void World::Step(float dt) {
    // 1. 速度积分与位置预测 (P0 -> P1)
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f || !b->IsAwake()) continue;
        Vector2 accel = b->getForce() * b->getInvMass() + m_gravity * b->getGravityScale();
        b->velocity += accel * dt;
        b->angularVelocity += b->torque * b->getInvInertia() * dt;

        b->SavePrevState(); // 记录当前位置为 P0
        b->position += b->velocity * dt; // 预测 P1
        b->rotation += b->angularVelocity * dt;

        b->ClearForce(); b->torque = 0.0f;
    }

    // 2. 定义局部宽相同步函数
    auto syncBP = [&](float currentDt) {
        for (Body* b : m_bodies) {
            if (b->getInvMass() == 0.0f || !b->IsAwake()) continue;
            b->updateAABB();
            AABB bpAABB = b->IsBullet() ? b->GetSweptAABB(currentDt) : b->GetAABB();
            m_broadPhase.MoveProxy(b->getProxyId(), bpAABB, b->velocity * currentDt);
        }
        };

    // 3. 初始同步与 TOI 收集
    syncBP(dt);
    UpdateAllContactsAndTOI(dt);

    // --- 4. [CCD 核心迭代] ---
    float remainingDt = dt;
    for (int subStep = 0; subStep < 4; ++subStep) {
        Contact* earliest = nullptr;
        TOIOutput bestOutput;
        float minAlpha = 1.0f;

        for (auto const& pair : m_contactMap) {
            Contact* c = pair.second;
            if (!c->m_bodyA->IsBullet() && !c->m_bodyB->IsBullet()) continue;
            // 重新算 TOI，确保使用当前缩短后的 remainingDt
            TOIInput input;
            input.bodyA = c->m_bodyA; input.bodyB = c->m_bodyB;
            input.dt = remainingDt; input.tolerance = 0.001f;
            TOIOutput out = TimeOfImpact::Solve(input);
            if (out.state == TOIOutput::Hit && out.alpha < minAlpha) {
                minAlpha = out.alpha; earliest = c; bestOutput = out;
            }
        }

        if (earliest && minAlpha < 1.0f) {
            Body* bA = earliest->m_bodyA; Body* bB = earliest->m_bodyB;

            // A. 回溯到撞击瞬间
            float safeAlpha = std::max(0.0f, minAlpha - 0.01f);
            bA->SetTransform(bA->GetTransform(safeAlpha, remainingDt));
            bB->SetTransform(bB->GetTransform(safeAlpha, remainingDt));

            // B. 使用 TOI 提供的法线强制反弹 (VN 计算是关键)
            Vector2 n = bestOutput.normal; // TOI 传回的法线
            Vector2 vRel = bA->velocity - bB->velocity;
            float vn = vRel.Dot(n);

            if (vn > 0.0f) { // 如果正在接近墙 (vA->右, n->右, 点积为正)
                float e = 0.5f;
                float j = (1.0f + e) * vn / (bA->getInvMass() + bB->getInvMass());
                bA->velocity -= n * (j * bA->getInvMass());
                bB->velocity += n * (j * bB->getInvMass());
                Logger::Info(">>> [CCD] BOUNCE! New VelX: " + std::to_string(bA->velocity.getX()));
            }

            // C. 【关键】：同步 P0 并消耗时间
            for (Body* b : m_bodies) b->SavePrevState();
            remainingDt *= (1.0f - minAlpha);

            // D. 为下一轮子步刷新宽相和 TOI
            syncBP(remainingDt);
            UpdateAllContactsAndTOI(remainingDt);
        }
        else break;
    }

    // 5. 最后执行离散解算（处理普通碰撞和堆叠）
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

void World::SolveTOI(Contact* contact, float dt) {
    Body* bodyA = contact->m_bodyA;
    Body* bodyB = contact->m_bodyB;
    float alpha = contact->m_toi;

    // --- 1. 双向回溯 (Backtracking both) ---
    // 无论物体是动态还是静态，都根据 alpha 比例回到撞击瞬间
    // 如果是静态物体，其 velocity 为 0，GetTransform 会返回原位，逻辑依然成立
    Transform xfA = bodyA->GetTransform(alpha, dt);
    Transform xfB = bodyB->GetTransform(alpha, dt);

    bodyA->SetPosition(xfA.p);
    bodyA->SetRotation(xfA.q);
    bodyB->SetPosition(xfB.p);
    bodyB->SetRotation(xfB.q);

    // --- 2. 刷新碰撞信息 ---
    contact->update();

    // --- 3. 拦截解算 ---
    if (contact->IsTouching()) {
        ImpulseSolver(contact->GetManifold());

        // 唤醒双方
        bodyA->setAwake(true);
        bodyB->setAwake(true);
    }

    // --- 4. 消耗掉这个 TOI ---
    contact->m_toi = 1.0f;
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

void World::UpdateAllContactsAndTOI(float dt) {
    // 1. 宽相寻找新对
    m_broadPhase.UpdatePairs([&](void* uA, void* uB) {
        HandleNewCollision(uA, uB, dt);
        });

    // 2. 更新已有对的状态和 TOI
    for (auto& pair : m_contactMap) {
        UpdateTOI(pair.second, dt);
    }
}

void World::UpdateNeighborsTOI(Body* b, float dt) {
    ContactEdge* ce = b->getContactList();
    while (ce) {
        UpdateTOI(ce->contact, dt);
        ce = ce->next;
    }
}

void World::HandleNewCollision(void* uA, void* uB, float dt) {
    Body* bodyA = static_cast<Body*>(uA);
    Body* bodyB = static_cast<Body*>(uB);

    // 1. 过滤：两个静态物体之间不需要碰撞处理
    if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) {
        return;
    }

    // 2. 保证 Key 的唯一性：让指针小的在前
    if (bodyA > bodyB) std::swap(bodyA, bodyB);
    auto key = std::make_pair(bodyA, bodyB);

    // 3. 检查是否已经存在该碰撞对
    if (m_contactMap.count(key)) {
        return; 
    }

    // 4. 创建新的持久化 Contact
    Contact* contact = new Contact(bodyA, bodyB);
    m_contactMap[key] = contact;
    AddContactToGraph(contact);

    contact->update();

    UpdateTOI(contact, dt);
}

void World::UpdateBroadPhase(float dt) {
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f || !b->IsAwake()) continue;
        b->updateAABB();
        AABB bpAABB = b->IsBullet() ? b->GetSweptAABB(dt) : b->GetAABB();
        m_broadPhase.MoveProxy(b->getProxyId(), bpAABB, b->velocity * dt);
    }
}