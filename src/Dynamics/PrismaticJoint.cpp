#include "PrismaticJoint.h"
#include "../common/Setting.h"
#include <cmath>

PrismaticJoint::PrismaticJoint(const PrismaticJointDef* def)
	:Joint(def),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_localXAxisA(def->localAxisA.normalize()),
	m_referenceAngle(def->referenceAngle),
	m_enableLimit(def->enableLimit),
	m_lowerTranslation(def->lowerTranslation),
	m_upperTranslation(def->upperTranslation),
	m_enableMotor(def->enableMotor),
	m_motorSpeed(def->motorSpeed),
	m_maxMotorForce(def->maxMotorForce)
{
	// 垂向轴 = 滑轨轴逆时针转 90°（Box2D 约定 n = (1,0)×t）
	m_localYAxisA = m_localXAxisA.getLeftNormal();
}

void PrismaticJoint::initVelocityConstraints(float dt)
{
	// 得到世界滑轨轴 t 与垂向轴 n（A 的局部轴旋转到世界系）
	m_axis = m_localXAxisA.rotate(m_bodyA->getRotation());
	m_perp = m_localYAxisA.rotate(m_bodyA->getRotation());

	Vector2 rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	Vector2 rB = m_localAnchorB.rotate(m_bodyB->getRotation());
	Vector2 pA = m_bodyA->getPosition() + rA;
	Vector2 pB = m_bodyB->getPosition() + rB;
	Vector2 d = pB - pA;          // 锚点间世界偏移
	Vector2 sA = d + rA;          // A 侧受力点相对 A 质心的臂（含锚点偏移）

	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// 轴向限位有效质量（1-DOF）
	m_a1 = Vector2::cross(sA, m_axis);
	m_a2 = Vector2::cross(rB, m_axis);
	m_axialMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
	if (m_axialMass > 0.0f) m_axialMass = 1.0f / m_axialMass;

	// 基础 2×2 有效质量：[垂向平移, 相对旋转]
	m_s1 = Vector2::cross(sA, m_perp);
	m_s2 = Vector2::cross(rB, m_perp);
	m_K11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
	m_K12 = iA * m_s1 + iB * m_s2;
	m_K22 = iA + iB;
	if (m_K22 == 0.0f) m_K22 = 1.0f; // 双静态刚体退化，防除零

	// 主约束无速度偏置（Box2D 2.4 风格，与 RevoluteJoint 一致）：
	// 偏置像阻尼一样会拖慢沿轨滑行（10 m/s 自由滑会被耗死），
	// 垂向漂移与旋转误差全部交给位置修正（每帧 4 次迭代）
	m_bias = Vector2(0.0f, 0.0f);

	// 限位状态机（与 Revolute 角度限位同构：角度 → 沿轴位移）
	float translation = m_axis.dot(d);
	if (m_enableLimit)
	{
		if (std::abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings::LINEAR_SLOP)
			m_limitState = LimitState::Equal;   // 上下限相等：双向锁死（刚性连接）
		else if (translation <= m_lowerTranslation)
			m_limitState = LimitState::AtLower; // 只能往 +t 推 (λ >= 0)
		else if (translation >= m_upperTranslation)
			m_limitState = LimitState::AtUpper; // 只能往 -t 推 (λ <= 0)
		else
		{
			m_limitState = LimitState::Inactive;
			m_limitImpulse = 0.0f; // 回到自由区间，清空限位冲量缓存
		}
	}
	else
	{
		m_limitState = LimitState::Inactive;
		m_limitImpulse = 0.0f;
	}

	// 限位激活时注入轴向 Baumgarte 偏置（目标轴向速度，把位移推回合法区间）
	m_limitBias = 0.0f;
	if (m_limitState != LimitState::Inactive)
	{
		float C = 0.0f;
		if (m_limitState == LimitState::Equal)
			C = translation - m_lowerTranslation;
		else if (m_limitState == LimitState::AtLower)
			C = translation - m_lowerTranslation; // 越界为负 → 偏置为正，往 +t 推
		else
			C = translation - m_upperTranslation; // 越界为正 → 偏置为负，往 -t 推
		C = std::max(-0.5f, std::min(0.5f, C));
		m_limitBias = -0.2f * C / dt;
	}

	// 马达每帧冲量钳位上限: λmax = dt · Fmax
	m_maxMotorImpulse = dt * m_maxMotorForce;

	// 热启动：上一帧累计冲量（基础 + 马达 + 限位）立即施加
	Vector2 P = m_perp * m_impulse.getX() + m_axis * (m_motorImpulse + m_limitImpulse);
	float LA = m_impulse.getX() * m_s1 + m_impulse.getY() + (m_motorImpulse + m_limitImpulse) * m_a1;
	float LB = m_impulse.getX() * m_s2 + m_impulse.getY() + (m_motorImpulse + m_limitImpulse) * m_a2;
	m_bodyA->setVelocity(m_bodyA->getVelocity() - P * mA);
	m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - iA * LA);
	m_bodyB->setVelocity(m_bodyB->getVelocity() + P * mB);
	m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + iB * LB);
}

void PrismaticJoint::solveVelocityConstraints()
{
	Vector2 vA = m_bodyA->getVelocity();
	float wA = m_bodyA->getAngularVelocity();
	Vector2 vB = m_bodyB->getVelocity();
	float wB = m_bodyB->getAngularVelocity();
	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// ---- 线性马达（1-DOF，双向对称饱和；与限位冲量绝对隔离）----
	// Equal 锁死时限位已把连杆焊死，跳过马达避免与刚性限位内耗
	if (m_enableMotor && m_limitState != LimitState::Equal)
	{
		float Cdot = m_axis.dot(vB - vA) + m_a2 * wB - m_a1 * wA;
		float impulse = m_axialMass * (m_motorSpeed - Cdot);
		float oldImpulse = m_motorImpulse;
		// 推力饱和钳位：累积马达冲量截断在 [-dt·Fmax, +dt·Fmax]
		m_motorImpulse = std::max(-m_maxMotorImpulse, std::min(m_maxMotorImpulse, m_motorImpulse + impulse));
		impulse = m_motorImpulse - oldImpulse;

		Vector2 P = m_axis * impulse;
		vA -= P * mA;
		wA -= iA * impulse * m_a1;
		vB += P * mB;
		wB += iB * impulse * m_a2;
	}

	// ---- 基础 2×2（垂向平移 + 相对旋转）：解 K·λ = -(Cdot + bias) ----
	Vector2 Cdot1(m_perp.dot(vB - vA) + m_s2 * wB - m_s1 * wA, wB - wA);
	Vector2 B = -(Cdot1 + m_bias);
	float det = m_K11 * m_K22 - m_K12 * m_K12;
	if (det != 0.0f)
	{
		float invDet = 1.0f / det;
		Vector2 lambda((m_K22 * B.getX() - m_K12 * B.getY()) * invDet,
			(m_K11 * B.getY() - m_K12 * B.getX()) * invDet);
		m_impulse += lambda;
		// 立即施加（后续限位求解读到最新速度 = 块 Gauss-Seidel）
		Vector2 P = m_perp * lambda.getX();
		vA -= P * mA;
		wA -= iA * (lambda.getX() * m_s1 + lambda.getY());
		vB += P * mB;
		wB += iB * (lambda.getX() * m_s2 + lambda.getY());
	}

	// ---- 轴向限位（1-DOF，单向截断）----
	if (m_enableLimit && m_limitState != LimitState::Inactive)
	{
		float Cdot2 = m_axis.dot(vB - vA) + m_a2 * wB - m_a1 * wA;
		float impulse = -m_axialMass * (Cdot2 - m_limitBias);
		float oldImpulse = m_limitImpulse;
		if (m_limitState == LimitState::Equal)
			m_limitImpulse += impulse;      // 双向等式：退化为刚性锁定
		else if (m_limitState == LimitState::AtLower)
			m_limitImpulse = std::max(m_limitImpulse + impulse, 0.0f); // 只能推 +t
		else
			m_limitImpulse = std::min(m_limitImpulse + impulse, 0.0f); // 只能推 -t
		float axialLambda = m_limitImpulse - oldImpulse;

		Vector2 P = m_axis * axialLambda;
		vA -= P * mA;
		wA -= iA * axialLambda * m_a1;
		vB += P * mB;
		wB += iB * axialLambda * m_a2;
	}

	m_bodyA->setVelocity(vA);
	m_bodyA->setAngularVelocity(wA);
	m_bodyB->setVelocity(vB);
	m_bodyB->setAngularVelocity(wB);
}

bool PrismaticJoint::solvePositionConstraints()
{
	float aA = m_bodyA->getRotation();
	float aB = m_bodyB->getRotation();
	Vector2 rA = m_localAnchorA.rotate(aA);
	Vector2 rB = m_localAnchorB.rotate(aB);
	Vector2 d = (m_bodyB->getPosition() + rB) - (m_bodyA->getPosition() + rA);
	Vector2 sA = d + rA;

	// 位置迭代每轮都用最新旋转重算世界轴（缓存力矩臂会过期，WeldJoint 同款教训）
	m_axis = m_localXAxisA.rotate(aA);
	m_perp = m_localYAxisA.rotate(aA);

	// 误差 C1 = (垂向平移误差, 相对角误差)
	Vector2 C1(m_perp.dot(d), aB - aA - m_referenceAngle);

	// 限位误差 C2：独立于速度求解的状态机，用当前位移直接判定——
	// 滑块在一帧内冲过限位时，位置修正当帧就能拦截（否则超界 = 冲击速度 × dt）
	float C2 = 0.0f;
	bool limitActive = false;
	if (m_enableLimit)
	{
		float translation = m_axis.dot(d);
		if (std::abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings::LINEAR_SLOP)
		{
			C2 = std::max(-0.5f, std::min(0.5f, translation - m_lowerTranslation));
			limitActive = true;
		}
		else if (translation <= m_lowerTranslation)
		{
			C2 = std::max(-0.5f, std::min(0.0f, translation - m_lowerTranslation));
			limitActive = true;
		}
		else if (translation >= m_upperTranslation)
		{
			C2 = std::max(0.0f, std::min(0.5f, translation - m_upperTranslation));
			limitActive = true;
		}
	}

	// 收敛判据（1e-6 门限，与 RevoluteJoint 锚点修正同款）：
	// 不能用 5mm 的 LINEAR_SLOP 当门限——每帧重力积分下沉 g·dt² ≈ 2.7mm 永远
	// 够不到修正门槛，滑块会以 2.7mm 幅度上下振荡（纯平移测试要求 < 1e-4）
	if (std::abs(C1.getX()) < 1e-6f &&
		std::abs(C1.getY()) < 1e-6f &&
		(!limitActive || std::abs(C2) < 1e-6f))
		return true;

	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// 基础 2×2 修正（新鲜力矩臂）
	m_s1 = Vector2::cross(sA, m_perp);
	m_s2 = Vector2::cross(rB, m_perp);
	float K11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
	float K12 = iA * m_s1 + iB * m_s2;
	float K22 = iA + iB;
	if (K22 == 0.0f) K22 = 1.0f;
	float det = K11 * K22 - K12 * K12;
	Vector2 P1(0.0f, 0.0f);
	if (det != 0.0f)
	{
		float invDet = 1.0f / det;
		// 解 K·P = -C1
		P1.setX(-(K22 * C1.getX() - K12 * C1.getY()) * invDet);
		P1.setY(-(K11 * C1.getY() - K12 * C1.getX()) * invDet);
	}

	// 轴向限位修正（1-DOF）
	float P2 = 0.0f;
	if (limitActive)
	{
		m_a1 = Vector2::cross(sA, m_axis);
		m_a2 = Vector2::cross(rB, m_axis);
		float axialMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
		if (axialMass > 0.0f)
			P2 = -C2 / axialMass; // "质量×位移"量纲冲量
	}

	// 合并修正：冲量量纲 → 乘 invMass/invInertia 才是位移（静态刚体自动原地不动）
	Vector2 P = m_perp * P1.getX() + m_axis * P2;
	float LA = P1.getX() * m_s1 + P1.getY() + P2 * m_a1;
	float LB = P1.getX() * m_s2 + P1.getY() + P2 * m_a2;

	m_bodyA->setPositionQuiet(m_bodyA->getPosition() - P * mA);
	m_bodyA->setRotationQuiet(aA - iA * LA);
	m_bodyB->setPositionQuiet(m_bodyB->getPosition() + P * mB);
	m_bodyB->setRotationQuiet(aB + iB * LB);

	return false;
}

Vector2 PrismaticJoint::getAnchorA() const
{
	return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 PrismaticJoint::getAnchorB() const
{
	return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}
