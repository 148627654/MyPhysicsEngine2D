#include "RopeJoint.h"
#include <algorithm>

RopeJoint::RopeJoint(const RopeJointDef* def)
	: Joint(def),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_maxLength(def->maxLength) {}

void RopeJoint::initVelocityConstraints(float dt)
{
	m_rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	m_rB = m_localAnchorB.rotate(m_bodyB->getRotation());

	Vector2 pA = m_bodyA->getPosition() + m_rA;
	Vector2 pB = m_bodyB->getPosition() + m_rB;

	Vector2 delta = pB - pA;
	float dist = delta.length();
	float C = dist - m_maxLength;
	if (C <= 0)
	{
		m_state = LimitState::Inactive;
		m_u = Vector2(0.0f, 0.0f);
		m_bias = 0.0f;
		m_impulse = 0.0f;
		m_mass = 0.0f;
		return;
	}
	else
	{
		m_state = LimitState::AtUpper;
		m_u = (dist > 1e-6f) ? delta / dist : Vector2(0.0f, 0.0f);// 归一化向量
		
		m_crAu = Vector2::cross(m_rA, m_u);
		m_crBu = Vector2::cross(m_rB, m_u);

		float K = m_bodyA->getInvMass() + m_bodyB->getInvMass()
			+ m_bodyA->getInvInertia() * m_crAu * m_crAu
			+ m_bodyB->getInvInertia() * m_crBu * m_crBu;
		m_mass = (K > 0.0f) ? 1.0f / K : 0.0f;
		m_bias = (dt > 0 ? 0.1 * C / dt : 0);
		m_bodyA->applyImpulse(-m_impulse * m_u, m_rA);
		m_bodyB->applyImpulse(m_impulse * m_u, m_rB);
	}
}

void RopeJoint::solveVelocityConstraints()
{
	// 1. 松弛态直接放行
	if (m_state == LimitState::Inactive)
		return;

	Vector2 vA = m_bodyA->getVelocity();
	float avA = m_bodyA->getAngularVelocity(); // float 标量
	Vector2 vB = m_bodyB->getVelocity();
	float avB = m_bodyB->getAngularVelocity(); // float 标量

	// 2. 轴向相对分离速度 Cdot
	float Cdot = m_u.dot(vB - vA) + m_crBu * avB - m_crAu * avA;

	// 3. 理论增量冲量
	float impulse = -m_mass * (Cdot + m_bias);

	// 4. 单侧拉力截断 (用 std::min 锁死在负数拉力区间，严禁正推力！)
	float oldImpulse = m_impulse;
	float newImpulse = std::min(oldImpulse + impulse, 0.0f);
	float actualImpulse = newImpulse - oldImpulse;
	m_impulse = newImpulse;

	// 5. 原子施加拉力冲量
	Vector2 P = actualImpulse * m_u;
	m_bodyA->applyImpulse(-P, m_rA);
	m_bodyB->applyImpulse(P, m_rB);
}
bool RopeJoint::solvePositionConstraints()
{
	// 位置迭代每轮用最新旋转重算力矩臂（上一帧缓存的 m_rA/m_rB 已过期）
	Vector2 rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	Vector2 rB = m_localAnchorB.rotate(m_bodyB->getRotation());
	Vector2 pA = m_bodyA->getPosition() + rA;
	Vector2 pB = m_bodyB->getPosition() + rB;
	Vector2 delta = pB - pA;
	float dist = delta.length();
	float C = dist - m_maxLength;
	// 1e-6 门限（与 Prismatic/Revolute 同款）：不能用 5mm 的 LINEAR_SLOP 当门限——
	// 每帧重力下沉 g·dt²≈2.7mm 的伸长永远够不到修正门槛，绳会停在 ~5mm 超长
	// （急停验收要求 < 0.003m）
	if (C < 1e-6f)
		return true;

	Vector2 u = (dist > 1e-6f) ? delta / dist : Vector2(0.0f, 0.0f);
	float crAu = Vector2::cross(rA, u);
	float crBu = Vector2::cross(rB, u);
	float K = m_bodyA->getInvMass() + m_bodyB->getInvMass()
		+ m_bodyA->getInvInertia() * crAu * crAu
		+ m_bodyB->getInvInertia() * crBu * crBu;
	float impulse = (K > 0.0f) ? -C / K : 0.0f;
	Vector2 P = impulse * u;
	m_bodyA->setPositionQuiet(m_bodyA->getPosition() - m_bodyA->getInvMass() * P);
	m_bodyA->setRotationQuiet(m_bodyA->getRotation() - m_bodyA->getInvInertia() * crAu * impulse);
	m_bodyA->updateAABB();
	m_bodyB->setPositionQuiet(m_bodyB->getPosition() + m_bodyB->getInvMass() * P);
	m_bodyB->setRotationQuiet(m_bodyB->getRotation() + m_bodyB->getInvInertia() * crBu * impulse);
	m_bodyB->updateAABB();

	// 修正后重算误差（精确解，单轮即收敛）
	Vector2 newDelta = (m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation()))
		- (m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation()));
	return newDelta.length() - m_maxLength < 1e-6f;
}
Vector2 RopeJoint::getAnchorA() const
{
	return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 RopeJoint::getAnchorB() const
{
	return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}
