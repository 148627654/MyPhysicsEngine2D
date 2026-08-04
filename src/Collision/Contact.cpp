#include "Contact.h"

Contact::Contact(Body* bodyA, Body* bodyB)
    : m_bodyA(bodyA), m_bodyB(bodyB), m_manifold(bodyA, bodyB), m_touching(false)
{
    m_nodeA.contact = this;
    m_nodeA.other = bodyB;
    m_nodeA.prev = nullptr;
    m_nodeA.next = nullptr;

    m_nodeB.contact = this;
    m_nodeB.other = bodyA;
    m_nodeB.prev = nullptr;
    m_nodeB.next = nullptr;

    m_islandFlag = false;
}

void Contact::PreSolve(float dt)
{
    Body* A = m_bodyA;
    Body* B = m_bodyB;
    Manifold& m = m_manifold;

    float invMassA = A->getInvMass();
    float invMassB = B->getInvMass();
    float invInertiaA = A->getInvInertia();
    float invInertiaB = B->getInvInertia();

    for (int i = 0; i < m.contactCount; ++i)
    {
        m.rA[i] = m.contacts[i] - A->GetPosition();
        m.rB[i] = m.contacts[i] - B->GetPosition();
        // 2. 计算法向有效质量 (Denominator of Impulse formula)
        // Formula: K = 1/mA + 1/mB + (rA x n)^2 / IA + (rB x n)^2 / IB
        float rnA = Vector2::Cross(m.rA[i], m.normal);
        float rnB = Vector2::Cross(m.rB[i], m.normal);
		float kNormal = invMassA + invMassB + rnA * rnA * invInertiaA + rnB * rnB * invInertiaB;
        m.massNormal[i] = (kNormal > 0.0f) ? 1.0f / kNormal : 0.0f;

        // 3. 计算切向有效质量
        Vector2 tangent = Vector2::Cross(m.normal, 1.0f); // 2D 垂直向量
        float rtA = Vector2::Cross(m.rA[i], tangent);
        float rtB = Vector2::Cross(m.rB[i], tangent);
        float kTangent = invMassA + invMassB + (rtA * rtA * invInertiaA) + (rtB * rtB * invInertiaB);
        m.massTangent[i] = (kTangent > 0.0f) ? 1.0f / kTangent : 0.0f;

        Vector2 P = (m.normal * m.impulseN[i]) + (tangent * m.impulseT[i]);

        // 立即改变速度，给解算器一个极好的“初始猜测值”
        A->ApplyImpulse(-P, m.rA[i]);
        B->ApplyImpulse(P, m.rB[i]);
    }
}
