#include "IsLand.h"
#include "../../include/physics/Dynamics/Solver.h" // 确保能访问到你之前的解算函数

IsLand::IsLand(int bodyCapacity, int contactCapacity) {
    m_bodies.reserve(bodyCapacity);
    m_contacts.reserve(contactCapacity);
}


void IsLand::Solve(const TimeStep& step, const Vector2& gravity) {
    float dt = step.dt;

    float minSleepTimer = 1000.0f;

    // 1. 能量监控
    for (Body* b : m_bodies) {
        if (b->getInvMass() == 0.0f) continue;

        // 如果该物体禁止休眠，或者当前是清醒的且动能大
        float linearVelocitySq = b->GetVelocity().LengthSquared();
        float angularVelocitySq = b->getAngularVelocity() * b->getAngularVelocity();

        if (!b->IsSleepAllow() ||
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
                b->SetVelocity(Vector2(0, 0)); // 物理平滑优化
                b->setAngularVelocity(0.0f);
            }
        }
        return;
    }
    //PreSolve
    for (Contact* c : m_contacts) {
        c->PreSolve(step.dt);
    }

    // --- 2. 冲量解算 (Velocity Constraints) ---
    for (int i = 0; i < step.velocityIterations; ++i) {
        for (Contact* c : m_contacts) {
            ImpulseSolver(c->GetManifold());
        }
    }

    // --- 4. 位置修正 (Position Constraints) ---
    for (int i = 0; i < step.positionIterations; ++i) {
        for (Contact* c : m_contacts) {
            PositionalCorrection(c->GetManifold());
        }
    }
}