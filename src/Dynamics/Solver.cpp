#include "../../include/physics/Collision/Manifold.h"
#include "../../include/physics/Common/Vector2.h"
#include <iostream>
#include "../../include/physics/Dynamics/Body.h"

void ImpulseSolver(Manifold& m)
{
	//**相对速度**：$\vec{v}_{rel} = \vec{v}_B - \vec{v}_A$。
	Body* a = m.bodyA;
	Body* b = m.bodyB;
	Vector2 v_rel = b->GetVelocity() - a->GetVelocity();
	//**法向速度** ：$v_{ normal } = \vec{ v }_{ rel } \cdot \vec{ n }$。
	float v_normal = v_rel.Dot(m.normal);

	if (v_normal > 0)return;

	//混合恢复系数公式通常采用：`e = std::min(A->e, B->e)`。
	float e = std::min(a->getRestitution(), b->getRestitution());

	//
	float invMassSum = a->getInvMass() + b->getInvMass();

	//$$j = \frac{ -(1 + e)(\vec{v}_{rel} \cdot \vec{n}) }{\frac{ 1 }{m_A} + \frac{ 1 }{m_B}}$$
	float j = -(1 + e) * v_normal / invMassSum;
	if (invMassSum < 0.0001f) return;

	//注意类型不匹配，需要把float转化成Vector2
	Vector2 impulseVector = m.normal * j;

	// 7. 应用冲量 (注意 A 是反向，B 是正向)
	a->ApplyImpulse(impulseVector * -1.0f);
	b->ApplyImpulse(impulseVector);
}