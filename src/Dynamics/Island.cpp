#include "Island.h"
#include "Joint.h"
#include "../../include/physics/Dynamics/Solver.h" // 确保能访问到你之前的解算函数

Island::Island(int bodyCapacity, int contactCapacity) {
    m_bodies.reserve(bodyCapacity);
    m_contacts.reserve(contactCapacity);
}


void Island::solve(const TimeStep& step, const Vector2& gravity) {
    float dt = step.dt;

    float minSleepTimer = 1000.0f;

    // 1. 能量监控
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f) continue;

        // 如果该物体禁止休眠，或者当前是清醒的且动能大
        float linearVelocitySq = b->getVelocity().lengthSquared();
        float angularVelocitySq = b->getAngularVelocity() * b->getAngularVelocity();

        // 【修复】仅当存在"正在接触"的实体接触（非触发器）时才允许速度归零。
        // 否则自由落体/抛射体在初始低速阶段每帧被清零，永远无法加速（从静止下落不动的 bug）
        // 注意必须检查 isTouching()：宽相的肥 AABB 会在形状真正接触前就创建 Contact，
        // 只查 !isTrigger() 会把"尚未接触"的接触当成实体接触，球照样被冻结
        bool hasSolidContact = false;
        for (ContactEdge* ce = b->getContactList(); ce != nullptr; ce = ce->next) {
            if (!ce->contact->isTrigger() && ce->contact->isTouching()) { hasSolidContact = true; break; }
        }

        if (hasSolidContact && linearVelocitySq < Settings::LinearSleepThreshold * 0.5f) {
            b->setVelocity(0);
            linearVelocitySq = 0.0f;
        }
        if (hasSolidContact && angularVelocitySq < Settings::AngularSleepThreshold * 0.5f) {
            b->setAngularVelocity(0.0f);
            angularVelocitySq = 0.0f;
        }
        if (!b->isSleepAllowed() ||
            linearVelocitySq > Settings::LinearSleepThreshold ||
            angularVelocitySq > Settings::AngularSleepThreshold)
        {
            //printf("Body Energy: %f | Threshold: %f\n", linearVelocitySq, Settings::LinearSleepThreshold);
            //printf("Body Energy: %f | Threshold: %f\n", angularVelocitySq, Settings::AngularSleepThreshold);
            b->setSleepTimer(0.0f);
            minSleepTimer = 0.0f;
        }
        else {
            b->setSleepTimer(b->getSleepTimer() + dt);
            minSleepTimer = std::min(minSleepTimer, b->getSleepTimer());
        }
    }

    // 2. 尝试集体入睡
    if (minSleepTimer >= Settings::TimeToSleep) {
        for (Body* b : m_bodies) {
            if (b->getInvMass() > 0.0f) {
                b->setAwake(false);
                b->setVelocity(Vector2(0, 0)); // 物理平滑优化
                b->setAngularVelocity(0.0f);
            }
        }
        return;
    }
    // 0. 初始化关节速度约束（每帧恰好一次）
    for (Joint* j : m_joints) {
        j->initVelocityConstraints(step.dt);
    }

    //preSolve
    for (Contact* c : m_contacts) {
        if (c->isTrigger()) continue; // 触发器不参与预热冲量
        c->preSolve(step.dt);
    }

    // --- 2. 冲量解算 (Velocity Constraints) ---
    for (int i = 0; i < step.velocityIterations; ++i) {
        for (Contact* c : m_contacts) {
            if (c->isTrigger()) continue; // 触发器不产生冲量
            impulseSolver(c->getManifold());
        }
        // 关节速度约束（每轮迭代各解一次）
        for (Joint* j : m_joints) {
            j->solveVelocityConstraints();
        }
    }

    // --- 4. 位置修正 (Position Constraints) ---
    for (int i = 0; i < step.positionIterations; ++i) {
        for (Contact* c : m_contacts) {
            if (c->isTrigger()) continue; // 触发器不做位置修正
            positionalCorrection(c->getManifold());
        }
        // 关节位置约束
        for (Joint* j : m_joints) {
            j->solvePositionConstraints();
        }
    }
}