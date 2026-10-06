#include "FrictionJoint.h"
#include <cmath>

FrictionJoint::FrictionJoint(const FrictionJointDef* def)
	: Joint(def),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_maxForce(def->maxForce),
	m_maxTorque(def->maxTorque) {}

void FrictionJoint::initVelocityConstraints(float dt)
{
	m_dt = dt;

	m_rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	m_rB = m_localAnchorB.rotate(m_bodyB->getRotation());

	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// 线 2×2 有效质量：锚点速度雅可比 J = [-I -rA_skew I rB_skew] 的 K 矩阵
	m_K11 = mA + mB + iA * m_rA.getY() * m_rA.getY() + iB * m_rB.getY() * m_rB.getY();
	m_K12 = -iA * m_rA.getX() * m_rA.getY() - iB * m_rB.getX() * m_rB.getY();
	m_K22 = mA + mB + iA * m_rA.getX() * m_rA.getX() + iB * m_rB.getX() * m_rB.getX();

	// 角有效质量
	m_angularMass = iA + iB;
	if (m_angularMass > 0.0f) m_angularMass = 1.0f / m_angularMass;

	// 热启动：上一帧累计的线/角摩擦冲量立即施加
	Vector2 P = m_linearImpulse;
	m_bodyA->setVelocity(m_bodyA->getVelocity() - P * mA);
	m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - iA * (Vector2::cross(m_rA, P) + m_angularImpulse));
	m_bodyB->setVelocity(m_bodyB->getVelocity() + P * mB);
	m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + iB * (Vector2::cross(m_rB, P) + m_angularImpulse));
}

void FrictionJoint::solveVelocityConstraints()
{
	Vector2 vA = m_bodyA->getVelocity();
	float wA = m_bodyA->getAngularVelocity();
	Vector2 vB = m_bodyB->getVelocity();
	float wB = m_bodyB->getAngularVelocity();
	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// ---- 角摩擦（1-DOF，标量区间截断 [-dt·τmax, +dt·τmax]）----
	{
		float Cdot = wB - wA;
		float impulse = -m_angularMass * Cdot;
		float oldImpulse = m_angularImpulse;
		float maxImpulse = m_dt * m_maxTorque;
		m_angularImpulse = std::max(-maxImpulse, std::min(maxImpulse, m_angularImpulse + impulse));
		impulse = m_angularImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// ---- 线摩擦（2-DOF，库仑圆截断）----
	{
		Vector2 Cdot = vB + Vector2(-wB * m_rB.getY(), wB * m_rB.getX())
			- vA - Vector2(-wA * m_rA.getY(), wA * m_rA.getX());

		// 解 K·λ = -Cdot
		float det = m_K11 * m_K22 - m_K12 * m_K12;
		Vector2 impulse(0.0f, 0.0f);
		if (det != 0.0f)
		{
			float invDet = 1.0f / det;
			Vector2 B = -Cdot;
			impulse.setX((m_K22 * B.getX() - m_K12 * B.getY()) * invDet);
			impulse.setY((m_K11 * B.getY() - m_K12 * B.getX()) * invDet);
		}

		Vector2 oldImpulse = m_linearImpulse;
		m_linearImpulse += impulse;

		// 库仑圆截断：平面各向摩擦上限相同，对向量**模长**截断，
		// 超限时沿当前方向径向等比缩放（不能分轴截断，否则对角方向摩擦力虚高 √2 倍）
		float maxImpulse = m_dt * m_maxForce;
		if (m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse)
			m_linearImpulse = m_linearImpulse.normalize() * maxImpulse;
		impulse = m_linearImpulse - oldImpulse;

		vA -= impulse * mA;
		wA -= iA * Vector2::cross(m_rA, impulse);
		vB += impulse * mB;
		wB += iB * Vector2::cross(m_rB, impulse);
	}

	m_bodyA->setVelocity(vA);
	m_bodyA->setAngularVelocity(wA);
	m_bodyB->setVelocity(vB);
	m_bodyB->setAngularVelocity(wB);
}

bool FrictionJoint::solvePositionConstraints()
{
	// 力学常识：摩擦力是耗散力，绝不拉回历史位置
	return true;
}

Vector2 FrictionJoint::getAnchorA() const
{
	return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 FrictionJoint::getAnchorB() const
{
	return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}
