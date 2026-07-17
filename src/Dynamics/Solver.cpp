#include "../../include/physics/Collision/Manifold.h"
#include "../../include/physics/Common/Vector2.h"
#include <iostream>
#include "../../include/physics/Dynamics/Body.h"
#include <CSVExporter.h>
#include "Solver.h"
#include "../../include/physics/Common/Setting.h"
#include <algorithm>

void ImpulseSolver(Manifold& m) {
    if (m.contacts.empty()) return;

    Body* a = m.bodyA;
    Body* b = m.bodyB;

    float invMassSum = a->getInvMass() + b->getInvMass();
    if (invMassSum < 0.0001f) return; // 两个都是静态物体

    // 1. 计算相对速度（包含旋转分量）
    Vector2 rA = m.contacts[0] - a->GetPosition();
    Vector2 rB = m.contacts[0] - b->GetPosition();

    // v_p = v_linear + w x r
    Vector2 vA_p = a->GetVelocity() + Vector2::Cross(a->getAngularVelocity(), rA);
    Vector2 vB_p = b->GetVelocity() + Vector2::Cross(b->getAngularVelocity(), rB);
    Vector2 v_rel = vB_p - vA_p;

    // 2. 计算法向速度
    float v_normal = v_rel.Dot(m.normal);
    
    Logger::Info("V_Rel_Normal: " + std::to_string(v_normal));
    // 【关键调试点】：如果物体穿透但不反弹，试着翻转这个判断
    // 逻辑：如果 v_normal < 0，说明物体正在相向运动（接近），才需要解算
    if (v_normal > 0.0f) return;

    // 3. 计算冲量标量 j
    float rAn = Vector2::Cross(rA, m.normal);
    float rBn = Vector2::Cross(rB, m.normal);
    float rotTerm = (rAn * rAn * a->getInvInertia()) + (rBn * rBn * b->getInvInertia());

    float e = std::min(a->getRestitution(), b->getRestitution());
    float j = -(1.0f + e) * v_normal / (invMassSum + rotTerm);

    // 4. 应用冲量
    Vector2 impulseVector = m.normal * j;

    // 平动更新
    a->ApplyImpulse(impulseVector * -1.0f);
    b->ApplyImpulse(impulseVector);

    // 转动更新（必须手动更新角速度）
    a->setAngularVelocity(a->getAngularVelocity() - Vector2::Cross(rA, impulseVector) * a->getInvInertia());
    b->setAngularVelocity(b->getAngularVelocity() + Vector2::Cross(rB, impulseVector) * b->getInvInertia());
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
