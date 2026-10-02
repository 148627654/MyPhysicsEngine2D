#include "SpringJoint.h"
#include "../common/Setting.h"
SpringJoint::SpringJoint(const SpringJointDef* def)
	:Joint(def),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_length(def->length){}

void SpringJoint::initVelocityConstraints(float dt)
{
	// 力矩臂与当前长度
	// 锚点相对质心的世界向量
	m_rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	m_rB = m_localAnchorB.rotate(m_bodyB->getRotation());
	Vector2 pA = m_bodyA->getPosition() + m_rA;
	Vector2 pB = m_bodyB->getPosition() + m_rB;

	Vector2 d = pB - pA;
	float dist = d.length();
	m_u = (dist > 1e-6f) ? d * (1.0f / dist) : Vector2(0.0f, 1.0f);
	// 算出两个力矩臂的叉积
	m_crAu = Vector2::cross(m_rA, m_u);
	m_crBu = Vector2::cross(m_rB, m_u);
	// 有效质量: k = invMassA + invMassB + invIA·(rA×u)² + invIB·(rB×u)²
	float K = m_bodyA->getInvMass() + m_bodyB->getInvMass() + m_bodyA->getInvInertia() * m_crAu * m_crAu + m_bodyB->getInvInertia() * m_crBu * m_crBu;
	float invK = (K > 0.0f) ? 1.0f / K : 0.0f;

	// 隐式软约束参数推导
	if (m_frequencyHz > 0.0f && dt > 0)//开启弹簧
	{
		// 震荡角频率
		float omega = 2.0f * Settings::PAI * m_frequencyHz;
		// 阻尼d和刚度k
		float d = 2.0f * invK * m_dampingRatio * omega;
		float k = invK * omega * omega;
		// 计算柔度参数 gamma 
		float softDenom = dt * (d + dt * k);
		m_gamma = (softDenom > 0.0f) ? 1.0f / softDenom : 0.0f;
		// 速度偏置 bias
		float C = dist - m_length;
		m_bias = C * dt * k * m_gamma;
		m_mass = 1 / (K + m_gamma);
	}
	else // 频率为0 退化成刚性杆
	{
		m_gamma = 0.0f;
		float C = dist - m_length;
		m_bias = (dt > 0.0f) ? (0.1f * C / dt) : 0.0f;
		m_mass = invK;
	}
	//热启动
	Vector2 P = m_u * m_impulse;
	m_bodyA->applyImpulse(-P, m_rA);
	m_bodyB->applyImpulse(P, m_rB);
}

void SpringJoint::solveVelocityConstraints()
{
	Body* a = m_bodyA;
	Body* b = m_bodyB;

	// 相对法向速度（杆长变化率）
	Vector2 vA = a->getVelocity() + Vector2::cross(a->getAngularVelocity(), m_rA);
	Vector2 vB = b->getVelocity() + Vector2::cross(b->getAngularVelocity(), m_rB);
	float Cdot = m_u.dot(vB - vA);

	// 刚性杆：把 Cdot 精确压到 0（单约束一次迭代即可收敛）
	float lambda = -m_mass * (Cdot+m_bias+m_gamma*m_impulse);
	m_impulse += lambda;

	Vector2 P = m_u * lambda;
	a->applyImpulse(-P, m_rA);
	b->applyImpulse(P, m_rB);
}

Vector2 SpringJoint::getAnchorA() const
{
	return m_localAnchorA;
}

Vector2 SpringJoint::getAnchorB() const
{
	return m_localAnchorB;
}
