#include "../../include/physics/Collision/Manifold.h"
#include "../../include/physics/Common/Vector2.h"
#include <iostream>
#include "../../include/physics/Dynamics/Body.h"
#include <CSVExporter.h>
#include "Solver.h"
#include "../../include/physics/Common/Setting.h"
#include <algorithm>

void impulseSolver(Manifold& m) {
    if (m.contactCount == 0) return;

    Body* A = m.bodyA;
    Body* B = m.bodyB;

    float invMassA = A->getInvMass();
    float invMassB = B->getInvMass();
    float invInertiaA = A->getInvInertia();
    float invInertiaB = B->getInvInertia();

    // 摩擦系数：按材质混合规则合成 (默认 Multiply = sqrt(a*b)，与原行为一致)
    const Physics2D::Material& matA = A->getShape()->material;
    const Physics2D::Material& matB = B->getShape()->material;
    float mu = Physics2D::Material::combine(matA.dynamicFriction, matB.dynamicFriction, matA.frictionCombine);


    for (int i = 0; i < m.contactCount; ++i) {
        // --- 1. 计算当前接触点的相对速度 ---
        // 注意：使用 preSolve 缓存的 rA[i] 和 rB[i]
        Vector2 vA_p = A->getVelocity() + Vector2::cross(A->getAngularVelocity(), m.rA[i]);
        Vector2 vB_p = B->getVelocity() + Vector2::cross(B->getAngularVelocity(), m.rB[i]);
        Vector2 v_rel = vB_p - vA_p;

        // --- 2. 法向增量冲量解算 ---
        float v_normal = v_rel.dot(m.normal);

        // 计算本次迭代需要的冲量增量 jn
        // 目标是把法向速度驱动到 bias（preSolve 中由恢复系数算出的反弹目标速度；
        // 无弹性时 bias = 0，行为与之前完全一致）
        float jn = -(v_normal - m.bias[i]) * m.massNormal[i];

        // 【关键】：累加并 Clamp 总冲量
        float oldImpulseN = m.impulseN[i];
        m.impulseN[i] = std::max(oldImpulseN + jn, 0.0f); // 保证总冲量永远 >= 0 (不会吸在一起)
        float actual_jn = m.impulseN[i] - oldImpulseN;   // 算出本轮真正施加的增量

        // 应用增量冲量
        Vector2 impulseN_vec = m.normal * actual_jn;
        A->applyImpulse(-impulseN_vec, m.rA[i]);
        B->applyImpulse(impulseN_vec, m.rB[i]);

        // --- 3. 切向增量冲量 (摩擦力) 解算 ---
        // 重新计算速度以包含刚刚法向冲量的影响
        vA_p = A->getVelocity() + Vector2::cross(A->getAngularVelocity(), m.rA[i]);
        vB_p = B->getVelocity() + Vector2::cross(B->getAngularVelocity(), m.rB[i]);
        v_rel = vB_p - vA_p;

        Vector2 tangent = Vector2::cross(m.normal, 1.0f); // 2D 切线
        float vt = v_rel.dot(tangent);
        float jt = -vt * m.massTangent[i];

        //累加并限制摩擦力总冲量 (库仑定律)
        float oldImpulseT = m.impulseT[i];
        float maxFriction = mu * m.impulseN[i]; // 基于当前总法向冲量限制
        m.impulseT[i] = std::max(-maxFriction, std::min(oldImpulseT + jt, maxFriction));
        float actual_jt = m.impulseT[i] - oldImpulseT; // 算出本轮真正施加的切向增量

        // 应用增量切向冲量
        Vector2 impulseT_vec = tangent * actual_jt;
        A->applyImpulse(-impulseT_vec, m.rA[i]);
        B->applyImpulse(impulseT_vec, m.rB[i]);
    }
}

//实现位置修正函数
/*
$$\text{correction} = \frac{\max(\text{penetration} - \text{slop}, 0)}
                    {\text{invMassA} + \text{invMassB}} \times \text{bias}$$
*/
void positionalCorrection(Manifold& m)
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

    // --- 【核心修复】：静默修改 position，避免触发 setAwake 导致物体永远无法入睡 ---
    // 【修复】法线由 A 指向 B：A 沿 -n 移动，B 沿 +n 移动（把两者分开）。
    // 之前 B 也沿 -n 移动，等于把 B 往 A 里推，物体会越陷越深
    if (m.bodyA->getInvMass() > 0.0f) {
        m.bodyA->setPositionQuiet(m.bodyA->getPosition() - correction_vector * m.bodyA->getInvMass());
    }

    if (m.bodyB->getInvMass() > 0.0f) {
        m.bodyB->setPositionQuiet(m.bodyB->getPosition() + correction_vector * m.bodyB->getInvMass());

    }

}