#include "GearJoint.h"
#include <cmath>

GearJoint::GearJoint(const GearJointDef* def)
	: Joint(def),
	m_joint1(def->joint1),
	m_joint2(def->joint2),
	m_ratio(def->ratio)
{
	m_typeA = m_joint1->getType();
	m_typeB = m_joint2->getType();

	// 提取 4 个关联刚体：基类 m_bodyA/m_bodyB = 两父关节的动态端（bodyB 侧），
	// m_bodyC/m_bodyD = 两父关节的固定端（bodyA 侧）
	m_bodyA = m_joint1->getBodyB();
	m_bodyB = m_joint2->getBodyB();
	m_bodyC = m_joint1->getBodyA();
	m_bodyD = m_joint2->getBodyA();

	// 基准标量：装配时刻的 coord1 + ratio·coord2
	m_constant = getCoordA() + m_ratio * getCoordB();
}

float GearJoint::getCoordA() const
{
	if (m_typeA == JointType::Revolute)
		return static_cast<RevoluteJoint*>(m_joint1)->getJointAngle();
	else
		return static_cast<PrismaticJoint*>(m_joint1)->getJointTranslation();
}

float GearJoint::getCoordB() const
{
	if (m_typeB == JointType::Revolute)
		return static_cast<RevoluteJoint*>(m_joint2)->getJointAngle();
	else
		return static_cast<PrismaticJoint*>(m_joint2)->getJointTranslation();
}

void GearJoint::recomputeJacobians()
{
	// --- 父关节 1（A=动态端, C=固定端）---
	if (m_typeA == JointType::Revolute)
	{
		// coord = θA - θC - ref：纯角自由度
		m_JvA = Vector2(0.0f, 0.0f);
		m_JwA = 1.0f;
		m_JvC = Vector2(0.0f, 0.0f);
		m_JwC = -1.0f;
	}
	else
	{
		PrismaticJoint* p = static_cast<PrismaticJoint*>(m_joint1);
		Vector2 axis = p->getLocalAxisA().rotate(m_bodyC->getRotation());
		Vector2 rA = p->getAnchorB() - m_bodyA->getPosition();
		Vector2 rC = p->getAnchorA() - m_bodyC->getPosition();
		m_JvA = axis;
		m_JwA = Vector2::cross(rA, axis);
		m_JvC = -axis;
		m_JwC = -Vector2::cross(rC, axis);
	}

	// --- 父关节 2（B=动态端, D=固定端）---
	if (m_typeB == JointType::Revolute)
	{
		m_JvB = Vector2(0.0f, 0.0f);
		m_JwB = 1.0f;
		m_JvD = Vector2(0.0f, 0.0f);
		m_JwD = -1.0f;
	}
	else
	{
		PrismaticJoint* p = static_cast<PrismaticJoint*>(m_joint2);
		Vector2 axis = p->getLocalAxisA().rotate(m_bodyD->getRotation());
		Vector2 rB = p->getAnchorB() - m_bodyB->getPosition();
		Vector2 rD = p->getAnchorA() - m_bodyD->getPosition();
		m_JvB = axis;
		m_JwB = Vector2::cross(rB, axis);
		m_JvD = -axis;
		m_JwD = -Vector2::cross(rD, axis);
	}
}

float GearJoint::computeK() const
{
	float ratio2 = m_ratio * m_ratio;
	return
		m_bodyA->getInvMass() * m_JvA.lengthSquared() + m_bodyA->getInvInertia() * m_JwA * m_JwA
		+ m_bodyC->getInvMass() * m_JvC.lengthSquared() + m_bodyC->getInvInertia() * m_JwC * m_JwC
		+ ratio2 * (m_bodyB->getInvMass() * m_JvB.lengthSquared() + m_bodyB->getInvInertia() * m_JwB * m_JwB
			+ m_bodyD->getInvMass() * m_JvD.lengthSquared() + m_bodyD->getInvInertia() * m_JwD * m_JwD);
}

void GearJoint::applyImpulse(float lambda)
{
	// Cdot = J_A·v_A - J_C·v_C + ratio·(J_B·v_B - J_D·v_D)
	// 施加 λ 后 ΔCdot = -λ·K，故各刚体速度变化 = -符号·λ·J·invM
	float sA = 1.0f, sC = -1.0f;
	float sB = m_ratio, sD = -m_ratio;

	m_bodyA->setVelocity(m_bodyA->getVelocity() - sA * lambda * m_JvA * m_bodyA->getInvMass());
	m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - sA * lambda * m_JwA * m_bodyA->getInvInertia());
	m_bodyC->setVelocity(m_bodyC->getVelocity() - sC * lambda * m_JvC * m_bodyC->getInvMass());
	m_bodyC->setAngularVelocity(m_bodyC->getAngularVelocity() - sC * lambda * m_JwC * m_bodyC->getInvInertia());
	m_bodyB->setVelocity(m_bodyB->getVelocity() - sB * lambda * m_JvB * m_bodyB->getInvMass());
	m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() - sB * lambda * m_JwB * m_bodyB->getInvInertia());
	m_bodyD->setVelocity(m_bodyD->getVelocity() - sD * lambda * m_JvD * m_bodyD->getInvMass());
	m_bodyD->setAngularVelocity(m_bodyD->getAngularVelocity() - sD * lambda * m_JwD * m_bodyD->getInvInertia());
}

void GearJoint::applyPositionCorrection(float lambda)
{
	float sA = 1.0f, sC = -1.0f;
	float sB = m_ratio, sD = -m_ratio;

	m_bodyA->setPositionQuiet(m_bodyA->getPosition() - sA * lambda * m_JvA * m_bodyA->getInvMass());
	m_bodyA->setRotationQuiet(m_bodyA->getRotation() - sA * lambda * m_JwA * m_bodyA->getInvInertia());
	m_bodyA->updateAABB();
	m_bodyC->setPositionQuiet(m_bodyC->getPosition() - sC * lambda * m_JvC * m_bodyC->getInvMass());
	m_bodyC->setRotationQuiet(m_bodyC->getRotation() - sC * lambda * m_JwC * m_bodyC->getInvInertia());
	m_bodyC->updateAABB();
	m_bodyB->setPositionQuiet(m_bodyB->getPosition() - sB * lambda * m_JvB * m_bodyB->getInvMass());
	m_bodyB->setRotationQuiet(m_bodyB->getRotation() - sB * lambda * m_JwB * m_bodyB->getInvInertia());
	m_bodyB->updateAABB();
	m_bodyD->setPositionQuiet(m_bodyD->getPosition() - sD * lambda * m_JvD * m_bodyD->getInvMass());
	m_bodyD->setRotationQuiet(m_bodyD->getRotation() - sD * lambda * m_JwD * m_bodyD->getInvInertia());
	m_bodyD->updateAABB();
}

void GearJoint::initVelocityConstraints(float dt)
{
	// 按当前位姿组装雅可比与有效质量
	recomputeJacobians();
	float K = computeK();
	m_mass = (K > 0.0f) ? 1.0f / K : 0.0f;

	// Baumgarte 几何误差偏置（β=0.2，仅真实误差启用）：
	// 【修复】不加门限时，位置修正前的浮点噪声级误差（C~2e-6，旋转累加的舍入游走）
	// 每几帧过 1e-6 门限一次，偏置把 ±1.5e-5 rad/s 的残差留进速度——传动比误差
	// 1e-5 验收必挂。真实误差（>0.1mm/0.1mrad 量级）才需要速度级回拉，
	// 微小误差由精确位置修正（1e-6 门限）当帧归零
	float C = getCoordA() + m_ratio * getCoordB() - m_constant;
	m_bias = (dt > 0.0f && std::abs(C) > 1e-4f) ? 0.2f * C / dt : 0.0f;

	// 热启动：上一帧累计传动冲量按雅可比权重回写 4 个刚体
	applyImpulse(m_impulse);
}

void GearJoint::solveVelocityConstraints()
{
	float Cdot =
		(m_JvA.dot(m_bodyA->getVelocity()) + m_JwA * m_bodyA->getAngularVelocity())
		- (m_JvC.dot(m_bodyC->getVelocity()) + m_JwC * m_bodyC->getAngularVelocity())
		+ m_ratio * ((m_JvB.dot(m_bodyB->getVelocity()) + m_JwB * m_bodyB->getAngularVelocity())
			- (m_JvD.dot(m_bodyD->getVelocity()) + m_JwD * m_bodyD->getAngularVelocity()));

	// 增量冲量（双向等式约束，无截断）：Δλ = mass·(Cdot + bias)
	float deltaLambda = m_mass * (Cdot + m_bias);
	m_impulse += deltaLambda;
	applyImpulse(deltaLambda);
}

bool GearJoint::solvePositionConstraints()
{
	// 实时重算当前位姿下的坐标误差（缓存雅可比会过期，每轮位置迭代重算）
	float C = getCoordA() + m_ratio * getCoordB() - m_constant;
	// 1e-6 门限（与其他关节同款）：传动比位移误差要求 1e-4 级，5mm slop 门限不够
	if (std::abs(C) < 1e-6f)
		return true;

	recomputeJacobians();
	float K = computeK();
	float lambda = (K > 0.0f) ? C / K : 0.0f;
	applyPositionCorrection(lambda);

	// 修正后重算误差
	return std::abs(getCoordA() + m_ratio * getCoordB() - m_constant) < 1e-6f;
}

Vector2 GearJoint::getAnchorA() const
{
	return m_joint1->getAnchorB();
}

Vector2 GearJoint::getAnchorB() const
{
	return m_joint2->getAnchorB();
}
