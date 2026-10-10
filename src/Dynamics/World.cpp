#include "World.h"
#include "DistanceJoint.h"
#include "FrictionJoint.h"
#include "GearJoint.h"
#include "PrismaticJoint.h"
#include "PulleyJoint.h"
#include "RevoluteJoint.h"
#include "RopeJoint.h"
#include "SpringJoint.h"
#include "WeldJoint.h"
#include "WheelJoint.h"
#include "../Collision/Collision.h"
#include "../Collision/Box.h"
#include "../Collision/Circle.h"
#include "../Collision/Capsule.h"
#include "../Utils/Logger.h"
#include "../../include/physics/Dynamics/Solver.h"
#include "../Collision/TimeOfImpact.h"
#include "../../include/physics/Utils/Profiler.h"

// 接触对 key 的统一构造：按 proxyId（加入世界的顺序）排序。
// 【修复】之前按指针大小排序，指针值随 ASLR/堆分配历史变化，
// 混合类型碰撞的流形法线方向会跨启动随机翻转（事件回调中的 normal 不稳定）。
// handleNewCollision 与 removeBody 必须共用本函数，规则漂移会导致 erase 落空 → 野指针。
static std::pair<Body*, Body*> makeContactKey(Body* a, Body* b) {
    if (a->getProxyId() > b->getProxyId()) std::swap(a, b);
    return std::make_pair(a, b);
}

void World::step(float dt) {
    // 处理上一帧之后外部（如回调中）调用的 destroyBody
    flushDestroyQueue();
    m_profiler.beginFrame();
    // --- 统计计数器 ---
    int awakeCount = 0;
    for (Body* b : m_bodies) if (b->isAwake()) awakeCount++;
    m_profiler.setCounter("Total Bodies", (int)m_bodies.size());
    m_profiler.setCounter("Awake Bodies", awakeCount);
    m_profiler.setCounter("Active Contacts", (int)m_contactMap.size());

    // 开始记录整个 step 的耗时
    m_profiler.start("Step Total");
    {
        ScopedTimer timer(m_profiler, "1. Integration");
        // 1. 速度积分与位置预测 (P0 -> P1)
        for (Body* b : m_bodies) {
            if (b->getInvMass() == 0.0f || !b->isAwake()) continue;
            Vector2 accel = b->getForce() * b->getInvMass() + m_gravity * b->getGravityScale();
            b->velocity += accel * dt;
            b->angularVelocity += b->torque * b->getInvInertia() * dt;

            b->savePrevState(); // 记录当前位置为 P0
            b->position += b->velocity * dt; // 预测 P1
            b->rotation += b->angularVelocity * dt;

            b->clearForce(); b->torque = 0.0f;
        }
    }
    

    // 2. 定义局部宽相同步函数
    auto syncBP = [&](float currentDt) {
        for (Body* b : m_bodies) {
            if (b->getInvMass() == 0.0f || !b->isAwake()) continue;
            b->updateAABB();
            AABB bpAABB = b->isBullet() ? b->getSweptAABB(currentDt) : b->getAABB();
            m_broadPhase.moveProxy(b->getProxyId(), bpAABB, b->velocity * currentDt);
        }
        };

    {
        ScopedTimer timer(m_profiler, "2. BroadPhase");
        // 3. 初始同步与 TOI 收集
        syncBP(dt);
        updateAllContactsAndTOI(dt);
    }

    {
        ScopedTimer timer(m_profiler, "2.5 Contact Events");
        // 接触生命周期事件：收集本帧所有正在接触的对，交给 ContactManager 推导 Enter/Stay/Exit
        for (auto& pair : m_contactMap) {
            Contact* c = pair.second;
            if (c->isTouching()) {
                m_contactManager.addContact(c->m_bodyA, c->m_bodyB, c->getManifold(), c->isTrigger());
            }
        }
        m_contactManager.updateStates();

        // 清理已经完全分离的接触对（AABB 不再重叠），防止接触对永久堆积
        destroySeparatedContacts();
    }

    {
        ScopedTimer timer(m_profiler, "3. CCD");
        // --- 4. [CCD 核心迭代] ---
        float remainingDt = dt;
        for (int subStep = 0; subStep < 4; ++subStep) {
            Contact* earliest = nullptr;
            TOIOutput bestOutput;
            float minAlpha = 1.0f;

            for (auto const& pair : m_contactMap) {
                Contact* c = pair.second;
                if (!c->m_bodyA->isBullet() && !c->m_bodyB->isBullet()) continue;
                // 重新算 TOI，确保使用当前缩短后的 remainingDt
                TOIInput input;
                input.bodyA = c->m_bodyA; input.bodyB = c->m_bodyB;
                input.dt = remainingDt; input.tolerance = 0.001f;
                TOIOutput out = TimeOfImpact::solve(input);
                if (out.state == TOIOutput::Hit && out.alpha < minAlpha) {
                    minAlpha = out.alpha; earliest = c; bestOutput = out;
                }
            }

            if (earliest && minAlpha < 1.0f) {
                Body* bA = earliest->m_bodyA; Body* bB = earliest->m_bodyB;

                // A. 回溯到撞击瞬间
                float safeAlpha = std::max(0.0f, minAlpha - 0.01f);
                bA->setTransform(bA->getTransform(safeAlpha, remainingDt));
                bB->setTransform(bB->getTransform(safeAlpha, remainingDt));

                // B. 使用 TOI 提供的法线强制反弹 (VN 计算是关键)
                Vector2 n = bestOutput.normal; // TOI 传回的法线
                Vector2 vRel = bA->velocity - bB->velocity;
                float vn = vRel.dot(n);

                if (vn > 0.0f) { // 如果正在接近墙 (vA->右, n->右, 点积为正)
                    float e = 0.5f;
                    float j = (1.0f + e) * vn / (bA->getInvMass() + bB->getInvMass());
                    bA->velocity -= n * (j * bA->getInvMass());
                    bB->velocity += n * (j * bB->getInvMass());
                    //Logger::info(">>> [CCD] BOUNCE! New VelX: " + std::to_string(bA->velocity.getX()));
                }


                // C. 【关键】：同步 P0 并消耗时间
                for (Body* b : m_bodies) b->savePrevState();
                remainingDt *= (1.0f - minAlpha);

                // D. 为下一轮子步刷新宽相和 TOI
                syncBP(remainingDt);
                updateAllContactsAndTOI(remainingDt);
            }
            else break;
        }
    }
    
    {
        ScopedTimer timer(m_profiler, "4. Island Solver");
        // 5. 最后执行离散解算（处理普通碰撞和堆叠）
        buildAndSolveIslands(dt);
    }

    {
        ScopedTimer timer(m_profiler, "5. Event dispatch");
        dispatchContactEvents();
    }

    // 帧末尾：处理延迟销毁队列（回调中 destroyBody 的物体在此刻真正移除）
    flushDestroyQueue();

    // 帧末尾：把本帧接触集移交为"上一帧"，供下一帧推导 Enter/Stay/Exit
    m_contactManager.endFrame();
    // 结束记录整个 step
    m_profiler.stop("Step Total");

    // [帧结束] 计算本帧平均值
    m_profiler.endFrame();
}

void World::dispatchContactEvents() {
    // 如果外部没有注册监听器，直接返回，零性能损耗
    if (m_contactListener == nullptr) return;

    const auto& records = m_contactManager.getLifecycleRecords();
    for (const auto& record : records) {
        if (record.isTrigger) {
            // --- 触发器事件打包与分发 ---
            TriggerEvent e;
            // 语义归一化：区分哪个是触发器传感器
            if (record.bodyA->getShape()->isTrigger) {
                e.triggerBody = record.bodyA;
                e.otherBody = record.bodyB;
            }
            else {
                e.triggerBody = record.bodyB;
                e.otherBody = record.bodyA;
            }

            switch (record.state) {
            case ContactState::Enter: m_contactListener->onTriggerEnter(e); break;
            case ContactState::Stay:  m_contactListener->onTriggerStay(e);  break;
            case ContactState::Exit:  m_contactListener->onTriggerExit(e);  break;
            }
        }
        else {
            // --- 物理实体碰撞事件打包与分发 ---
            CollisionEvent e;
            e.bodyA = record.bodyA;
            e.bodyB = record.bodyB;
            e.normal = record.manifold.normal;
            for (int i = 0; i < record.manifold.contactCount; ++i) {
                e.contacts.push_back(record.manifold.contacts[i]);
            }
            e.maxImpulse = 0.0f; // 如果解算器有记录冲量可填入

            switch (record.state) {
            case ContactState::Enter: m_contactListener->onCollisionEnter(e); break;
            case ContactState::Stay:  m_contactListener->onCollisionStay(e);  break;
            case ContactState::Exit:  m_contactListener->onCollisionExit(e);  break;
            }
        }
    }
}

void World::addContactToGraph(Contact* c) {
    Body* bodyA = c->m_bodyA;
    Body* bodyB = c->m_bodyB;

    c->m_nodeA.next = bodyA->m_contactList;
    if (bodyA->m_contactList) bodyA->m_contactList->prev = &c->m_nodeA;
    bodyA->m_contactList = &c->m_nodeA;

    c->m_nodeB.next = bodyB->m_contactList;
    if (bodyB->m_contactList) bodyB->m_contactList->prev = &c->m_nodeB;
    bodyB->m_contactList = &c->m_nodeB;
}

void World::removeContactFromGraph(Contact* c) {
    // Body A 侧
    if (c->m_nodeA.prev) c->m_nodeA.prev->next = c->m_nodeA.next;
    if (c->m_nodeA.next) c->m_nodeA.next->prev = c->m_nodeA.prev;
    if (c->m_bodyA->m_contactList == &c->m_nodeA) c->m_bodyA->m_contactList = c->m_nodeA.next;

    // Body B 侧
    if (c->m_nodeB.prev) c->m_nodeB.prev->next = c->m_nodeB.next;
    if (c->m_nodeB.next) c->m_nodeB.next->prev = c->m_nodeB.prev;
    if (c->m_bodyB->m_contactList == &c->m_nodeB) c->m_bodyB->m_contactList = c->m_nodeB.next;
}

void World::solveTOI(Contact* contact, float dt) {
    Body* bodyA = contact->m_bodyA;
    Body* bodyB = contact->m_bodyB;
    float alpha = contact->m_toi;

    // --- 1. 双向回溯 (Backtracking both) ---
    // 无论物体是动态还是静态，都根据 alpha 比例回到撞击瞬间
    // 如果是静态物体，其 velocity 为 0，getTransform 会返回原位，逻辑依然成立
    Transform xfA = bodyA->getTransform(alpha, dt);
    Transform xfB = bodyB->getTransform(alpha, dt);

    bodyA->setPosition(xfA.p);
    bodyA->setRotation(xfA.q);
    bodyB->setPosition(xfB.p);
    bodyB->setRotation(xfB.q);

    // --- 2. 刷新碰撞信息 ---
    contact->update();

    // --- 3. 拦截解算 ---
    if (contact->isTouching() && !contact->isTrigger()) {
        impulseSolver(contact->getManifold());

        // 唤醒双方
        bodyA->setAwake(true);
        bodyB->setAwake(true);
    }

    // --- 4. 消耗掉这个 TOI ---
    contact->m_toi = 1.0f;
}

void World::removeBody(Body* body) {
    if (body == nullptr) return;

    // 级联销毁与该刚体相连的所有关节（同时清空对方刚体的关节列表）
    while (!body->getJointList().empty()) {
        destroyJoint(body->getJointList().back());
    }

    // 通知接触管理器清除所有涉及该 Body 的历史缓存，杜绝野指针
    m_contactManager.onBodyDestroyed(body);

    // 1. 【核心修复】：从碰撞图中彻底抹除该物体
    ContactEdge* ce = body->getContactList();
    while (ce != nullptr) {
        Contact* contact = ce->contact;
        ce = ce->next; // 提前保存下一个，因为当前的 contact 马上要被删了

        // A. 唤醒邻居（防止悬空）
        contact->m_bodyA->setAwake(true);
        contact->m_bodyB->setAwake(true);

        // B. 从全局 map 中移除（key 规则必须与 handleNewCollision 完全一致，
        //    否则 erase 落空，接触残留野指针下一帧崩溃）
        auto key = makeContactKey(contact->m_bodyA, contact->m_bodyB);
        m_contactMap.erase(key);

        // C. 从两个物体的双向链表中安全摘除（调用你之前写的辅助函数）
        removeContactFromGraph(contact);

        // D. 销毁 Contact 对象
        delete contact;
    }

    // 2. 从宽相树中移除代理
    if (body->getProxyId() != -1) {
        m_broadPhase.destroyProxy(body->getProxyId());
    }

    // 3. 从世界物体列表中移除
    auto it = std::find(m_bodies.begin(), m_bodies.end(), body);
    if (it != m_bodies.end()) {
        m_bodies.erase(it);
    }

    // 4. 最后才真正释放内存
    delete body;
}

void World::buildAndSolveIslands(float dt) {
    // 1. 初始化标记
    for (Body* b : m_bodies) b->m_islandFlag = false;
    for (auto& pair : m_contactMap) pair.second->m_islandFlag = false;
    // 重置关节的岛屿遍历标记（用专门的 m_islandFlag，不碰 collideConnected 的语义）
    for (Joint* j : m_joints) j->m_islandFlag = false;

    TimeStep step{};
    step.dt = dt;
    // 【调整】速度迭代 8→10、位置迭代 3→4：
    // 多刚体紧密串联（布娃娃/闭环四连杆）时提升迭代可显著增强关节刚度，
    // 减小限位超界与闭环漂移（Day 13 规格建议值）
    step.velocityIterations = 10;
    step.positionIterations = 4;

    int islandCount = 0; // 诊断：记录本帧生成的岛屿总数
    m_islands.clear();   // 重建本帧岛屿集合（供测试检查 DFS 连通性）

    // 2. 遍历所有物体寻找“种子”
    for (Body* seed : m_bodies) {
        //检查该岛屿是否符合休眠条件。
        if (seed->m_islandFlag || seed->getInvMass() == 0.0f||!seed->isAwake()) continue;

        // --- DFS 诊断日志：发现新岛屿 ---
        islandCount++;

        m_islands.emplace_back(m_bodies.size(), m_contactMap.size());
        Island& island = m_islands.back();

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

            island.add(b);
            bodiesInIsland++; // 计数

            for (ContactEdge* ce = b->getContactList(); ce; ce = ce->next) {
                Contact* contact = ce->contact;

                if (contact->m_islandFlag || !contact->isTouching()) continue;

                island.add(contact);
                contact->m_islandFlag = true;
                contactsInIsland++; // 计数

                Body* other = ce->other;
                if (other->getInvMass() > 0.0f) {
                    if (other->m_islandFlag) continue; // 已经处理过的动态物体，跳过

                    if (!other->isAwake()) {
                        other->setAwake(true);
                    }

                    // 标记并入栈，继续向外传染岛屿
                    other->m_islandFlag = true;
                    stack.push_back(other);
                }
                
                else {
                    island.add(other);
                }
            }

            for (Joint* joint : b->getJointList()) {
                // 如果这个关节已经加入过某个岛屿了，跳过
                if (joint->m_islandFlag) continue;

                // 将关节加入当前岛屿解算器，并打上标记
                island.add(joint);
                joint->m_islandFlag = true;

                // 寻找关节另一端的对向刚体：谁不是 b，谁就是对方！
                Body* other = (joint->getBodyA() == b) ? joint->getBodyB() : joint->getBodyA();

                // 如果对方是动态物体 (invMass > 0)
                if (other->getInvMass() > 0.0f) {
                    if (other->m_islandFlag) continue; // 已经处理过，跳过

                    // 关节唤醒机制：如果被连结的物体在睡觉，立刻叫醒它！
                    if (!other->isAwake()) {
                        other->setAwake(true);
                    }

                    // 标记并入栈，让 DFS 继续顺着对方身上的关节和碰撞向外传染
                    other->m_islandFlag = true;
                    stack.push_back(other);
                }
                else {
                    // 如果对方是静态物体（比如钉在地上的锚点），加入岛屿作为支撑边界
                    island.add(other);
                }
            }
        }
        // 4. 解算
        island.solve(step, m_gravity);
    }
}

void World::rayCast(Vector2 p1, Vector2 p2) {
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
		void* userData = m_broadPhase.getUserData(proxyId);
		Body* body = static_cast<Body*>(userData);

		// 2. 执行真正的窄相检测 (Box 或 Circle 的 rayCast)
		RayCastOutput output;
		bool hit = body->getShape()->rayCast(&output, subInput, body->getPosition(), body->getRotation());

		if (hit) {
			// 3. 既然撞到了真实的形状，我们打印详细信息
			printf("[RAY HIT] Body at (%f, %f)\n", body->getPosition().getX(), body->getPosition().getY());
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
	m_broadPhase.rayCast(input, broadPhaseCallback);
}

void World::wakeNeighbors(Body* body)
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

void World::updateTOI(Contact* c, float dt) {
    if (!c->m_bodyA->isBullet() && !c->m_bodyB->isBullet()) return;
    
    TOIInput input;
    input.bodyA = c->m_bodyA;
    input.bodyB = c->m_bodyB;
    input.dt = dt;
    input.tolerance = Settings::LINEAR_SLOP;

    TOIOutput output = TimeOfImpact::solve(input);

    // 【核心修正】：只要判定为 Hit 或 Overlapped，都要记录 alpha
    if (output.state == TOIOutput::Hit || output.state == TOIOutput::Overlapped) {
        c->m_toi = output.alpha;
        //Logger::info(">>> [TOI SUCCESS] Alpha: " + std::to_string(c->m_toi));
    }
    else {
        // --- 增加这行诊断日志 ---
        //Logger::info(">>> [TOI FAILED] State: " + std::to_string(output.state));
        c->m_toi = 1.0f;
    }
}

void World::updateAllContactsAndTOI(float dt) {
    // 1. 宽相寻找新对
    m_broadPhase.updatePairs([&](void* uA, void* uB) {
        handleNewCollision(uA, uB, dt);
        });

    // 2. 每帧刷新已有接触的几何状态（touching / 流形）并更新 TOI
    // 【修复】之前已有接触从不重新求值：若接触在肥 AABB 重叠但形状未真正接触时创建，
    // touching 永远保持 false，物体将直接穿透（如高速球穿过地面）
    for (auto& pair : m_contactMap) {
        pair.second->update();
        updateTOI(pair.second, dt);
    }
}

void World::updateNeighborsTOI(Body* b, float dt) {
    ContactEdge* ce = b->getContactList();
    while (ce) {
        updateTOI(ce->contact, dt);
        ce = ce->next;
    }
}

void World::handleNewCollision(void* uA, void* uB, float dt) {
    Body* bodyA = static_cast<Body*>(uA);
    Body* bodyB = static_cast<Body*>(uB);

    // 1. 过滤：两个静态物体之间不需要碰撞处理
    if (bodyA->getInvMass() == 0.0f && bodyB->getInvMass() == 0.0f) {
        return;
    }

    // 1.5 collideConnected 过滤：关节直连且明确禁止碰撞的刚体对不产生接触
    //（布娃娃/链条装配时骨骼端部会天然交叉，不滤掉会与关节约束互相打架）
    for (Joint* j : bodyA->getJointList()) {
        if ((j->getBodyA() == bodyB && j->getBodyB() == bodyA) ||
            (j->getBodyA() == bodyA && j->getBodyB() == bodyB)) {
            if (!j->getCollideConnected()) return;
        }
    }

    // 2. 统一 key 构造（proxyId 排序，见 makeContactKey）
    auto key = makeContactKey(bodyA, bodyB);

    // 3. 检查是否已经存在该碰撞对
    if (m_contactMap.count(key)) {
        return; 
    }

    // 4. 创建新的持久化 Contact
    Contact* contact = new Contact(bodyA, bodyB);
    m_contactMap[key] = contact;
    addContactToGraph(contact);

    contact->update();

    updateTOI(contact, dt);
}

void World::updateBroadPhase(float dt) {
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f || !b->isAwake()) continue;
        b->updateAABB();
        AABB bpAABB = b->isBullet() ? b->getSweptAABB(dt) : b->getAABB();
        m_broadPhase.moveProxy(b->getProxyId(), bpAABB, b->velocity * dt);
    }
}

void World::destroySeparatedContacts() {
    // 销毁已经完全分离的接触对（形状不接触且 AABB 不再重叠），
    // 否则 m_contactMap 会永久堆积，"接触对数量清零"永远不可能发生
    for (auto it = m_contactMap.begin(); it != m_contactMap.end();) {
        Contact* c = it->second;
        Body* bA = c->m_bodyA;
        Body* bB = c->m_bodyB;
        if (!c->isTouching() && !Collision::aabbVsAabb(bA->getAABB(), bB->getAABB())) {
            removeContactFromGraph(c);
            delete c;
            it = m_contactMap.erase(it);
        }
        else {
            ++it;
        }
    }
}

Joint* World::createJoint(const JointDef& def) {
    Joint* joint = nullptr;

    switch (def.type) {
    case JointType::Distance:
        joint = new DistanceJoint(&static_cast<const DistanceJointDef&>(def));
        break;
    case JointType::Spring:
        joint = new SpringJoint(&static_cast<const SpringJointDef&>(def));
        break;
    case JointType::Revolute:
        joint = new RevoluteJoint(&static_cast<const RevoluteJointDef&>(def));
        break;
    case JointType::Prismatic:
        joint = new PrismaticJoint(&static_cast<const PrismaticJointDef&>(def));
        break;
    case JointType::Weld:
        joint = new WeldJoint(&static_cast<const WeldJointDef&>(def));
        break;
    case JointType::Friction:
        joint = new FrictionJoint(&static_cast<const FrictionJointDef&>(def));
        break;
    case JointType::Rope:
        joint = new RopeJoint(&static_cast<const RopeJointDef&>(def));
        break;
    case JointType::Pulley:
        joint = new PulleyJoint(&static_cast<const PulleyJointDef&>(def));
        break;
    case JointType::Gear:
        joint = new GearJoint(&static_cast<const GearJointDef&>(def));
        break;
    case JointType::Wheel:
        joint = new WheelJoint(&static_cast<const WheelJointDef&>(def));
        break;
    default:
        return nullptr; // 未知类型
    }

    add(joint);
    return joint;
}

void World::add(Joint* joint) {
    m_joints.push_back(joint);
    joint->m_bodyA->addJoint(joint);
    joint->m_bodyB->addJoint(joint);
    // 齿轮关节额外把父关节的 A 侧刚体（C/D）也挂上关节链表：
    // 图论遍历能索引到全部 4 个刚体，destroyBody 级联销毁也能覆盖齿轮
    if (joint->getType() == JointType::Gear) {
        GearJoint* gear = static_cast<GearJoint*>(joint);
        gear->getBodyC()->addJoint(gear);
        gear->getBodyD()->addJoint(gear);
    }
}

void World::destroyJoint(Joint* joint) {
    if (joint == nullptr) return;

    // 1. 从关联刚体的关节列表移除（Body::removeJoint 内部有存在性检查）
    joint->m_bodyA->removeJoint(joint);
    joint->m_bodyB->removeJoint(joint);
    // 齿轮关节额外注册了父关节的 A 侧刚体（C/D），必须一并摘除，
    // 否则 destroyBody 时列表里残留悬空指针 → 二次释放
    if (joint->getType() == JointType::Gear) {
        GearJoint* gear = static_cast<GearJoint*>(joint);
        gear->getBodyC()->removeJoint(gear);
        gear->getBodyD()->removeJoint(gear);
    }

    // 2. 从世界关节列表移除
    auto it = std::find(m_joints.begin(), m_joints.end(), joint);
    if (it != m_joints.end()) {
        m_joints.erase(it);
    }

    // 3. 级联断链防护：被销毁的关节若是某个齿轮关节的父关节，齿轮必须同步销毁
    //    （齿轮持有父关节裸指针，悬空 = 野指针崩溃）。先收集再销毁，避免迭代器失效
    std::vector<Joint*> cascaded;
    for (Joint* j : m_joints) {
        if (j->getType() == JointType::Gear) {
            GearJoint* gear = static_cast<GearJoint*>(j);
            if (gear->getJoint1() == joint || gear->getJoint2() == joint) {
                cascaded.push_back(j);
            }
        }
    }
    for (Joint* j : cascaded) {
        destroyJoint(j);
    }

    // 4. 释放内存
    delete joint;
}

void World::destroyBody(Body* body) {
    // 延迟销毁：刚体只入队（绝不在回调执行中途碰任何数据结构），
    // 真正移除在 step 末尾的 flushDestroyQueue() 中完成
    if (body == nullptr) return;

    // 去重，防止重复入队导致 double delete
    for (Body* b : m_destroyQueue) {
        if (b == body) return;
    }

    // 级联销毁关节立即执行（关节不被求解器长期持有，立即销毁安全），
    // 这样调用方在 destroyBody 之后立刻就能观察到"关节数归零"
    while (!body->getJointList().empty()) {
        destroyJoint(body->getJointList().back());
    }

    m_destroyQueue.push_back(body);
}

void World::flushDestroyQueue() {
    if (m_destroyQueue.empty()) return;

    for (Body* body : m_destroyQueue) {
        removeBody(body);
    }
    m_destroyQueue.clear();
}

// ================= 工厂接口实现 =================

Body* World::createBody(Shape* shape, float x, float y, float density) {
    Body* body = new Body(shape, x, y, density);
    addBody(body);
    return body;
}

Body* World::createBox(float w, float h, float x, float y, float density,
    const Physics2D::Material& mat) {
    return createBody(new Box(w, h, mat), x, y, density);
}

Body* World::createCircle(float radius, float x, float y, float density,
    const Physics2D::Material& mat) {
    return createBody(new Circle(radius, mat), x, y, density);
}

Body* World::createCapsule(float radius, float length, float x, float y, float density,
    const Physics2D::Material& mat) {
    return createBody(new Capsule(radius, length, mat), x, y, density);
}