#include "PulleyJoint.h"


PulleyJoint::PulleyJoint(const PulleyJointDef* def)
	: Joint(def),
	m_groundAnchorA(def->groundAnchorA),
	m_groundAnchorB(def->groundAnchorB),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_ratio(def->ratio)
{
	// 总等效绳长 L = lengthA + ratio·lengthB：以装配时刻的实际绳长为准
	// （Box2D 风格——def 里的 lengthA/lengthB 字段仅作文档参考，不参与约束）
	Vector2 pA0 = def->bodyA->getPosition() + def->localAnchorA.rotate(def->bodyA->getRotation());
	Vector2 pB0 = def->bodyB->getPosition() + def->localAnchorB.rotate(def->bodyB->getRotation());
	float lengthA0 = def->groundAnchorA.distance(pA0);
	float lengthB0 = def->groundAnchorB.distance(pB0);
	m_constant = lengthA0 + def->ratio * lengthB0;
}

void PulleyJoint::initVelocityConstraints(float dt)
{
	m_rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	m_rB = m_localAnchorB.rotate(m_bodyB->getRotation());
	Vector2 pA = m_bodyA->getPosition() + m_rA;
	Vector2 pB = m_bodyB->getPosition() + m_rB;
	float lengthA = m_groundAnchorA.distance(pA);
	float lengthB = m_groundAnchorB.distance(pB);
	// 归一化单位向量（存入成员——速度/位置求解都要用）
	m_uA = (lengthA > 1e-6f) ? (pA - m_groundAnchorA) / lengthA : Vector2(0.0f, 0.0f);
	m_uB = (lengthB > 1e-6f) ? (pB - m_groundAnchorB) / lengthB : Vector2(0.0f, 0.0f);
	// 组装含有ratio^2 的有效质量分母K
	float K = m_bodyA->getInvMass() + m_bodyA->getInvInertia() * Vector2::cross(m_rA, m_uA) * Vector2::cross(m_rA, m_uA)
		+ m_ratio * m_ratio * (m_bodyB->getInvMass() + m_bodyB->getInvInertia() * Vector2::cross(m_rB, m_uB) * Vector2::cross(m_rB, m_uB));
	m_mass = (K > 0.0f) ? 1.0f / K : 0.0f;
	// 【修复】主约束无速度偏置（Box2D 2.4 风格）：偏置按修正前的 C 算出，
	// 位置修正归零后刚体残留 ±0.1·C/dt 的蠕动速度（实测 ±0.016 m/s），
	// "平衡态速度恒为 0" 验收必挂。位置误差交给精确位置修正（1e-6 门限）
	m_bias = 0.0f;
	// 热启动：施加上次迭代的冲量
	m_bodyA->applyImpulse(-m_impulse * m_uA, m_rA);
	m_bodyB->applyImpulse(-m_ratio * m_impulse * m_uB, m_rB);
}

void PulleyJoint::solveVelocityConstraints()
{
	// 计算两锚点沿各自绳索轴向的速度投影并按照ratio加权求和，得到约束速度Cdot
	Vector2 vA = m_bodyA->getVelocity();
	float avA = m_bodyA->getAngularVelocity();
	Vector2 vB = m_bodyB->getVelocity();
	float avB = m_bodyB->getAngularVelocity();
	m_crAu = Vector2::cross(m_rA, m_uA);
	m_crBu = Vector2::cross(m_rB, m_uB);
	float Cdot = m_uA.dot(vA) + m_crAu * avA + m_ratio * (m_uB.dot(vB) + m_crBu * avB);
	// 【修复】冲量符号：施加 λ 后 ΔCdot = -λ·K，要让 Cdot → -bias 必须
	// λ = (Cdot + bias)/K。原符号 λ = -(Cdot+bias)/K 会把误差放大成 2·Cdot 发散
	float detalambda = m_mass * (Cdot + m_bias);
	m_impulse += detalambda;
	// 施加冲量写回两个刚体的速度
	m_bodyA->applyImpulse(-detalambda * m_uA, m_rA);
	m_bodyB->applyImpulse(-m_ratio * detalambda * m_uB, m_rB);
}

bool PulleyJoint::solvePositionConstraints()
{
	// 实时重新计算当前位姿下的最新绳长与方向（缓存的方向向量会过期）
	Vector2 rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	Vector2 rB = m_localAnchorB.rotate(m_bodyB->getRotation());
	Vector2 pA = m_bodyA->getPosition() + rA;
	Vector2 pB = m_bodyB->getPosition() + rB;
	float lengthA = m_groundAnchorA.distance(pA);
	float lengthB = m_groundAnchorB.distance(pB);
	Vector2 uA = (lengthA > 1e-6f) ? (pA - m_groundAnchorA) / lengthA : Vector2(0.0f, 0.0f);
	Vector2 uB = (lengthB > 1e-6f) ? (pB - m_groundAnchorB) / lengthB : Vector2(0.0f, 0.0f);
	float C = lengthA + m_ratio * lengthB - m_constant;
	// 1e-6 门限（与其他关节同款）：5mm slop 门限会让恒等式误差停在毫米级
	// （运动传递比验收要求总绳长恒等式误差 < 1e-4）
	if (C < 1e-6f)
		return true;

	// 计算瞬时质量，求解位置修正冲量
	// 【修复】符号：C>0 = 绳过长要缩短，修正方向应把锚点拉回吊点（原符号反向拉长）
	float K = m_bodyA->getInvMass() + m_bodyA->getInvInertia() * Vector2::cross(rA, uA) * Vector2::cross(rA, uA)
		+ m_ratio * m_ratio * (m_bodyB->getInvMass() + m_bodyB->getInvInertia() * Vector2::cross(rB, uB) * Vector2::cross(rB, uB));

	float detalambda = (K > 0.0f) ? C / K : 0.0f;
	// 平移世界坐标并旋转角度修正位置
	m_bodyA->setPositionQuiet(m_bodyA->getPosition() + (-detalambda * uA) * m_bodyA->getInvMass());
	m_bodyA->setRotationQuiet(m_bodyA->getRotation() + m_bodyA->getInvInertia() * Vector2::cross(rA, -detalambda * uA));
	m_bodyA->updateAABB();
	m_bodyB->setPositionQuiet(m_bodyB->getPosition() + (-m_ratio * detalambda * uB) * m_bodyB->getInvMass());
	m_bodyB->setRotationQuiet(m_bodyB->getRotation() + m_bodyB->getInvInertia() * Vector2::cross(rB, -m_ratio * detalambda * uB));
	m_bodyB->updateAABB();

	// 修正后重算误差
	Vector2 nA = m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
	Vector2 nB = m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
	return m_groundAnchorA.distance(nA) + m_ratio * m_groundAnchorB.distance(nB) - m_constant < 1e-6f;
}
Vector2 PulleyJoint::getAnchorA() const
{
	return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 PulleyJoint::getAnchorB() const
{
	return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}

float PulleyJoint::getLengthA() const
{
	// 计算 BodyA 到滑轮 A 的绳索长度 返回PulleyJoint的成员变量m_groundAnchorA和BodyA的锚点位置之间的距离
	return m_groundAnchorA.distance(getAnchorA());
}

float PulleyJoint::getLengthB() const
{
	return m_groundAnchorB.distance(getAnchorB());
}
