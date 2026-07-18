#include "../../include/physics/Collision/Manifold.h"
#include "../../include/physics/Common/Vector2.h"
#include <iostream>
#include "../../include/physics/Dynamics/Body.h"
#include <CSVExporter.h>
#include "Solver.h"
#include "../../include/physics/Common/Setting.h"
#include <algorithm>

void ImpulseSolver(Manifold& m) {
    if (m.contactCount == 0) return;

    Body* A = m.bodyA;
    Body* B = m.bodyB;

    float invMassA = A->getInvMass();
    float invMassB = B->getInvMass();
    float invInertiaA = A->getInvInertia();
    float invInertiaB = B->getInvInertia();
    float invMassSum = invMassA + invMassB;

    float e = std::min(A->getRestitution(), B->getRestitution());
    float mu = std::sqrt(A->getFriction() * B->getFriction());

    for (int i = 0; i < m.contactCount; ++i) {
        Vector2 rA = m.contacts[i] - A->GetPosition();
        Vector2 rB = m.contacts[i] - B->GetPosition();

        // 1. 法向冲量解算
        Vector2 vA_p = A->GetVelocity() + Vector2::Cross(A->getAngularVelocity(), rA);
        Vector2 vB_p = B->GetVelocity() + Vector2::Cross(B->getAngularVelocity(), rB);
        Vector2 v_rel = vB_p - vA_p;

        float v_normal = v_rel.Dot(m.normal);
        if (v_normal > 0) continue; // 正在分离

        float rAn = Vector2::Cross(rA, m.normal);
        float rBn = Vector2::Cross(rB, m.normal);
        float shareDenominator = invMassSum + (rAn * rAn * invInertiaA) + (rBn * rBn * invInertiaB);

        float jn = -(1.0f + e) * v_normal / shareDenominator;
        jn /= (float)m.contactCount;

        Vector2 impulseN = m.normal * jn;
        A->ApplyImpulse(-impulseN, rA);
        B->ApplyImpulse(impulseN, rB); // 假设你有这个函数或者直接操作

        // 2. 切向冲量 (摩擦力) 解算
        // 必须重新计算速度，因为法向冲量刚刚改变了它
        vA_p = A->GetVelocity() + Vector2::Cross(A->getAngularVelocity(), rA);
        vB_p = B->GetVelocity() + Vector2::Cross(B->getAngularVelocity(), rB);
        v_rel = vB_p - vA_p;

        Vector2 tangent = v_rel - (m.normal * v_rel.Dot(m.normal));
        if (tangent.LengthSquared() > 0.0001f) {
            tangent = tangent.Normalize();
        }
        else {
            continue;
        }

        float vt = v_rel.Dot(tangent);
        float rAt = Vector2::Cross(rA, tangent);
        float rBt = Vector2::Cross(rB, tangent);
        float frictionDenominator = invMassSum + (rAt * rAt * invInertiaA) + (rBt * rBt * invInertiaB);

        float jt = -vt / frictionDenominator;
        jt /= (float)m.contactCount;

        // --- C++11 适配：库仑定律限制 ---
        float maxFriction = jn * mu;
        if (jt > maxFriction) jt = maxFriction;
        else if (jt < -maxFriction) jt = -maxFriction;

        Vector2 impulseT = tangent * jt;
        A->ApplyImpulse(-impulseT, rA);
        B->ApplyImpulse(impulseT, rB);
    }
}

//实现位置修正函数
/*
$$\text{correction} = \frac{\max(\text{penetration} - \text{slop}, 0)}
                    {\text{invMassA} + \text{invMassB}} \times \text{bias}$$
*/
void PositionalCorrection(Manifold& m)
{
    float slot = ::Settings::PENETRATION_ALLOWANCE;
    float bias = ::Settings::BIAS;

    float suminvmass = (m.bodyA)->getInvMass() + (m.bodyB)->getInvMass();
    if (suminvmass < 0.0001f) return;
    float correction = std::max(m.penetration - slot, 0.0f) / suminvmass * bias;

    Vector2 posA = m.bodyA->GetPosition() - correction * ((m.bodyA)->getInvMass()) * m.normal;
    m.bodyA->SetPosition(posA);

    Vector2 posB = m.bodyB->GetPosition() + correction * ((m.bodyB)->getInvMass()) * m.normal;
    m.bodyB->SetPosition(posB);
}
