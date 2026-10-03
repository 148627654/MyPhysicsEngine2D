#include "RevoluteJoint.h"
#include "../common/Vector2.h"
#include "../common/Setting.h"
RevoluteJoint::RevoluteJoint(const RevoluteJointDef* def)
	: Joint(def),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_enableMotor(def->enableMotor),
	m_motorSpeed(def->motorSpeed),
	m_maxMotorTorque(def->maxMotorTorque),
	m_enableLimit(def->enableLimit),
	m_lowerAngle(def->lowerAngle),
	m_upperAngle(def->upperAngle),
	m_referenceAngle(def->referenceAngle){}

void RevoluteJoint::initVelocityConstraints(float dt)
{
	// 得到力矩臂向量
	m_rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	m_rB = m_localAnchorB.rotate(m_bodyB->getRotation());
	Vector2 pA = m_bodyA->getPosition() + m_rA;
	Vector2 pB = m_bodyB->getPosition() + m_rB;
	// 核心矩阵推到 K11 K12 K22
	// K11
	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// 直接用分量相乘，速度极快：
	// 【修复】Vector2 的 x/y 是私有成员，必须用 getX()/getY() 访问
	m_K11 = mA + mB + iA * m_rA.getY() * m_rA.getY() + iB * m_rB.getY() * m_rB.getY();
	m_K22 = mA + mB + iA * m_rA.getX() * m_rA.getX() + iB * m_rB.getX() * m_rB.getX();
	m_K12 = -iA * m_rA.getX() * m_rA.getY() - iB * m_rB.getX() * m_rB.getY();
	// 【修复】不再向速度约束注入位置偏置（Box2D 2.4 风格）：
	// 偏置像阻尼一样会在 ~30 帧内耗死摆动（违反"自由摆动"），
	// 还会制造 ~0.048 的误差均衡（达不到 < 1e-4 的锚点重合要求）。
	// 位置误差完全交给 solvePositionConstraints（每帧 3 次迭代）处理
	m_bias = Vector2(0.0f, 0.0f);
	// 二维冲量热启动
	Vector2 P = m_impulse;
	m_bodyA->applyImpulse(-P, m_rA);
	m_bodyB->applyImpulse(P, m_rB);

	// ================= 角自由度（1-DOF）=================
	// 角有效质量: m_ang = 1 / (iA + iB)  （复用上方已声明的 iA/iB）
	m_angularMass = (iA + iB > 0.0f) ? 1.0f / (iA + iB) : 0.0f;

	// 当前相对角度与限位状态机
	float angle = getJointAngle();
	if (m_enableLimit) {
		if (std::abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings::ANGULAR_SLOP) {
			m_limitState = LimitState::Equal;   // 上下限相等：双向锁死（焊接）
		}
		else if (angle <= m_lowerAngle) {
			m_limitState = LimitState::AtLower; // 只能往大推 (λ >= 0)
		}
		else if (angle >= m_upperAngle) {
			m_limitState = LimitState::AtUpper; // 只能往小推 (λ <= 0)
		}
		else {
			m_limitState = LimitState::Inactive;
			m_limitImpulse = 0.0f; // 回到自由区间，清空限位冲量缓存
		}
	}
	else {
		m_limitState = LimitState::Inactive;
		m_limitImpulse = 0.0f;
	}

	// 限位激活时注入角度 Baumgarte 偏置（目标角速度，把角度推回合法区间）
	m_limitBias = 0.0f;
	if (m_limitState != LimitState::Inactive) {
		float C = 0.0f;
		if (m_limitState == LimitState::Equal) {
			C = angle - m_lowerAngle;
		}
		else if (m_limitState == LimitState::AtLower) {
			C = angle - m_lowerAngle; // 越界时为负，偏置为正 → 把角度推大
		}
		else {
			C = angle - m_upperAngle; // 越界时为正，偏置为负 → 把角度推小
		}
		C = std::max(-0.5f, std::min(0.5f, C));
		m_limitBias = -0.2f * C / dt;
	}

	// 角冲量热启动：把上一帧的 (马达 + 限位) 累计冲量作为扭矩立即施加
	float angularImpulse = m_motorImpulse + m_limitImpulse;
	if (angularImpulse != 0.0f) {
		m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - iA * angularImpulse);
		m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + iB * angularImpulse);
	}

	// 马达每帧冲量钳位上限: λmax = dt · τmax
	m_maxMotorImpulse = dt * m_maxMotorTorque;
}

void RevoluteJoint::solveVelocityConstraints()
{
	// 计算相对速度
	Vector2 vA = m_bodyA->getVelocity();
	Vector2 vB = m_bodyB->getVelocity();
	
	Vector2 wA = Vector2(-m_bodyA->getAngularVelocity() * m_rA.getY(), m_bodyA->getAngularVelocity() * m_rA.getX());
	Vector2 wB = Vector2(-m_bodyB->getAngularVelocity() * m_rB.getY(), m_bodyB->getAngularVelocity() * m_rB.getX());
	Vector2 Cdot = vB + wB - vA - wA;
	// 组装目标误差向量 B
	Vector2 B = -(Cdot + m_bias);
	// 求解 2x2 线性方程组 K * impulse = B

	float det = m_K11 * m_K22 - m_K12 * m_K12;
	if (det != 0.0f)
	{
		float invDet = 1.0f / det;
		Vector2 impulse(
			(m_K22 * B.getX() - m_K12 * B.getY()) * invDet,
			(m_K11 * B.getY() - m_K12 * B.getX()) * invDet
		);
		m_impulse += impulse;
		m_bodyA->applyImpulse(-impulse, m_rA);
		m_bodyB->applyImpulse(impulse, m_rB);
	}

	// ================= 马达约束（1-DOF）=================
	// 在 ±τmax·dt 内驱动相对角速度趋近 motorSpeed；等角锁死时限位接管，马达让位
	if (m_enableMotor && m_limitState != LimitState::Equal) {
		float iA = m_bodyA->getInvInertia();
		float iB = m_bodyB->getInvInertia();

		// 速度误差: Cdot = (ωB - ωA) - ωmotor
		float Cdot = (m_bodyB->getAngularVelocity() - m_bodyA->getAngularVelocity()) - m_motorSpeed;
		float impulse = -m_angularMass * Cdot;

		// 扭矩钳位：累积马达冲量限制在 [-λmax, +λmax]
		float oldImpulse = m_motorImpulse;
		m_motorImpulse = std::max(-m_maxMotorImpulse, std::min(m_maxMotorImpulse, m_motorImpulse + impulse));
		impulse = m_motorImpulse - oldImpulse;

		m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - iA * impulse);
		m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + iB * impulse);
	}

	// ================= 限位约束（1-DOF，单向截断）=================
	if (m_enableLimit && m_limitState != LimitState::Inactive) {
		float iA = m_bodyA->getInvInertia();
		float iB = m_bodyB->getInvInertia();

		float Cdot = m_bodyB->getAngularVelocity() - m_bodyA->getAngularVelocity();
		float impulse = -m_angularMass * (Cdot - m_limitBias);

		float oldImpulse = m_limitImpulse;
		if (m_limitState == LimitState::Equal) {
			// 双向等式约束：锁死角度，无需截断
			m_limitImpulse += impulse;
		}
		else if (m_limitState == LimitState::AtLower) {
			// 单向不等式：只能产生使角度增大的正冲量
			m_limitImpulse = std::max(m_limitImpulse + impulse, 0.0f);
		}
		else {
			// 单向不等式：只能产生使角度减小的负冲量
			m_limitImpulse = std::min(m_limitImpulse + impulse, 0.0f);
		}
		impulse = m_limitImpulse - oldImpulse;

		m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - iA * impulse);
		m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + iB * impulse);
	}

}

bool RevoluteJoint::solvePositionConstraints()
{
	// 旋转后的世界力矩臂！
	Vector2 rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	Vector2 rB = m_localAnchorB.rotate(m_bodyB->getRotation());

	Vector2 pA = m_bodyA->getPosition() + rA;
	Vector2 pB = m_bodyB->getPosition() + rB;

	Vector2 C = pB - pA;
	float positionError = C.length();

	// 锚点重合修正（误差足够小时跳过，但【绝不能】用早退跳过后面的角度限位修正）
	if (positionError > 1e-6f)
	{
		// 【修复】单步修正钳制，防止极端初始错位时位置修正爆炸
		C.setX(std::max(-0.2f, std::min(0.2f, C.getX())));
		C.setY(std::max(-0.2f, std::min(0.2f, C.getY())));

		float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
		float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

		// 2. 全部基于世界力矩臂 rA 和 rB 计算有效质量矩阵 K
		float K11 = mA + mB + iA * rA.getY() * rA.getY() + iB * rB.getY() * rB.getY();
		float K22 = mA + mB + iA * rA.getX() * rA.getX() + iB * rB.getX() * rB.getX();
		float K12 = -iA * rA.getX() * rA.getY() - iB * rB.getX() * rB.getY();

		float det = K11 * K22 - K12 * K12;
		if (std::abs(det) > 1e-6f)
		{
			float invDet = 1.0f / det;

			// 3. 注意负号：解 K * P = -C
			Vector2 impulse(
				-(K22 * C.getX() - K12 * C.getY()) * invDet,
				-(K11 * C.getY() - K12 * C.getX()) * invDet
			);

			// 4. 直接修改坐标与旋转角度
			// 【修复】用静默 setter（setPositionQuiet/setRotationQuiet）：
			// 之前用 setPosition/setRotation 会唤醒刚体并重置睡眠计时，
			m_bodyA->setPositionQuiet(m_bodyA->getPosition() - impulse * mA);
			m_bodyB->setPositionQuiet(m_bodyB->getPosition() + impulse * mB);

			// 5. 使用世界力矩臂与冲量做叉乘更新角度
			m_bodyA->setRotationQuiet(m_bodyA->getRotation() - iA * Vector2::cross(rA, impulse));
			m_bodyB->setRotationQuiet(m_bodyB->getRotation() + iB * Vector2::cross(rB, impulse));
		}
	}

	// ================= 角度限位位置修正 =================
	if (m_enableLimit && m_limitState != LimitState::Inactive) {
		float iA = m_bodyA->getInvInertia();
		float iB = m_bodyB->getInvInertia();
		if (m_angularMass > 0.0f) {
			float angle = getJointAngle();
			float C = 0.0f;
			if (m_limitState == LimitState::Equal) {
				// 双向：把角度钳回 lower（= upper）
				C = std::max(-0.5f, std::min(0.5f, angle - m_lowerAngle));
			}
			else if (m_limitState == LimitState::AtLower) {
				// 单向：只修正负向越界（把角度推回 >= lower）
				C = std::max(-0.5f, std::min(0.0f, angle - m_lowerAngle));
			}
			else {
				// 单向：只修正正向越界（把角度推回 <= upper）
				C = std::max(0.0f, std::min(0.5f, angle - m_upperAngle));
			}

			if (std::abs(C) > 1e-6f) {
				float impulse = -m_angularMass * C;
				m_bodyA->setRotationQuiet(m_bodyA->getRotation() - iA * impulse);
				m_bodyB->setRotationQuiet(m_bodyB->getRotation() + iB * impulse);
			}
		}
	}

	return false;
}

Vector2 RevoluteJoint::getAnchorA() const
{
	// 【修复】基类接口约定返回"锚点世界坐标"（局部锚点经旋转+平移）
	return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 RevoluteJoint::getAnchorB() const
{
	// 【修复】同上：之前直接返回局部锚点，两个常量相减永远得不到重合误差
	return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}
