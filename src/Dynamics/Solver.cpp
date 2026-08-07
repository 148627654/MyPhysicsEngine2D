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

    // 摩擦系数
    float mu = std::sqrt(A->getFriction() * B->getFriction());

    for (int i = 0; i < m.contactCount; ++i) {
        // --- 1. 计算当前接触点的相对速度 ---
        // 注意：使用 PreSolve 缓存的 rA[i] 和 rB[i]
        Vector2 vA_p = A->GetVelocity() + Vector2::Cross(A->getAngularVelocity(), m.rA[i]);
        Vector2 vB_p = B->GetVelocity() + Vector2::Cross(B->getAngularVelocity(), m.rB[i]);
        Vector2 v_rel = vB_p - vA_p;

        // --- 2. 法向增量冲量解算 ---
        float v_normal = v_rel.Dot(m.normal);

        // 计算本次迭代需要的冲量增量 jn
        // 注意：这里不再乘 (1+e)，弹力已经在 PreSolve 的初始速度里处理了（见下文 Tip）
        float jn = -v_normal * m.massNormal[i];

        // 【关键】：累加并 Clamp 总冲量
        float oldImpulseN = m.impulseN[i];
        m.impulseN[i] = std::max(oldImpulseN + jn, 0.0f); // 保证总冲量永远 >= 0 (不会吸在一起)
        float actual_jn = m.impulseN[i] - oldImpulseN;   // 算出本轮真正施加的增量

        // 应用增量冲量
        Vector2 impulseN_vec = m.normal * actual_jn;
        A->ApplyImpulse(-impulseN_vec, m.rA[i]);
        B->ApplyImpulse(impulseN_vec, m.rB[i]);

        // --- 3. 切向增量冲量 (摩擦力) 解算 ---
        // 重新计算速度以包含刚刚法向冲量的影响
        vA_p = A->GetVelocity() + Vector2::Cross(A->getAngularVelocity(), m.rA[i]);
        vB_p = B->GetVelocity() + Vector2::Cross(B->getAngularVelocity(), m.rB[i]);
        v_rel = vB_p - vA_p;

        Vector2 tangent = Vector2::Cross(m.normal, 1.0f); // 2D 切线
        float vt = v_rel.Dot(tangent);
        float jt = -vt * m.massTangent[i];

        //累加并限制摩擦力总冲量 (库仑定律)
        float oldImpulseT = m.impulseT[i];
        float maxFriction = mu * m.impulseN[i]; // 基于当前总法向冲量限制
        m.impulseT[i] = std::max(-maxFriction, std::min(oldImpulseT + jt, maxFriction));
        float actual_jt = m.impulseT[i] - oldImpulseT; // 算出本轮真正施加的切向增量

        // 应用增量切向冲量
        Vector2 impulseT_vec = tangent * actual_jt;
        A->ApplyImpulse(-impulseT_vec, m.rA[i]);
        B->ApplyImpulse(impulseT_vec, m.rB[i]);
    }
}

//实现位置修正函数
/*
$$\text{correction} = \frac{\max(\text{penetration} - \text{slop}, 0)}
                    {\text{invMassA} + \text{invMassB}} \times \text{bias}$$
*/
void PositionalCorrection(Manifold& m)
{
    float slot = Settings::PENETRATION_ALLOWANCE;
    float bias = Settings::BIAS;
    // 【新增】：单次迭代的最大修正上限 (建议 0.05m ~ 0.1m)
    static constexpr float MAX_CORRECTION = 0.1f;

    float suminvmass = m.bodyA->getInvMass() + m.bodyB->getInvMass();
    if (suminvmass < Settings::EPSILON) return;

    // 计算修正量
    float correction_magnitude = std::max(m.penetration - slot, 0.0f) / suminvmass * bias;

    // 【核心修复】：限幅，防止物体被“炸”飞或瞬移过墙
    correction_magnitude = std::min(correction_magnitude, MAX_CORRECTION);

    Vector2 correction_vector = m.normal * correction_magnitude;

    // --- 【核心修复】：使用直接修改 position，避免触发 setAwake ---
    if (m.bodyA->getInvMass() > 0.0f) {
        m.bodyA->SetPosition(m.bodyA->GetPosition() - correction_vector * m.bodyA->getInvMass());
    }

    if (m.bodyB->getInvMass() > 0.0f) {
        m.bodyB->SetPosition(m.bodyB->GetPosition() - correction_vector * m.bodyB->getInvMass());

    }

}