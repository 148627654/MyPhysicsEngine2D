#include "WheelJoint.h"
#include "../common/Setting.h"
#include <cmath>

WheelJoint::WheelJoint(const WheelJointDef* def)
	:Joint(def),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_localXAxisA(def->localAxisA.normalize()),
	m_enableMotor(def->enableMotor),
	m_motorSpeed(def->motorSpeed),
	m_maxMotorTorque(def->maxMotorTorque)
{
	// 正交轴 n = s^⊥ = (-s_y, s_x)（悬挂轴逆时针转 90°，即左法线）
	m_localYAxisA = m_localXAxisA.getLeftNormal();
}

void WheelJoint::initVelocityConstraints(float dt)
{
	// 世界悬挂轴 s 与正交轴 n（随底盘 A 旋转）
	m_axis = m_localXAxisA.rotate(m_bodyA->getRotation());
	m_perp = m_localYAxisA.rotate(m_bodyA->getRotation());

	// 世界力矩臂与锚点
	m_rA = m_localAnchorA.rotate(m_bodyA->getRotation());
	m_rB = m_localAnchorB.rotate(m_bodyB->getRotation());
	Vector2 pA = m_bodyA->getPosition() + m_rA;
	Vector2 pB = m_bodyB->getPosition() + m_rB;
	Vector2 d = pB - pA;

	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// 力矩臂投影：sA = rA × s, sB = rB × s, nA = rA × n, nB = rB × n
	m_sA = Vector2::cross(m_rA, m_axis);
	m_sB = Vector2::cross(m_rB, m_axis);
	m_nA = Vector2::cross(m_rA, m_perp);
	m_nB = Vector2::cross(m_rB, m_perp);

	// 2×2 轮轴有效质量矩阵 K
	m_K11 = mA + mB + iA * m_nA * m_nA + iB * m_nB * m_nB;
	m_K12 = iA * m_nA * m_sA + iB * m_nB * m_sB;
	m_K22 = mA + mB + iA * m_sA * m_sA + iB * m_sB * m_sB;

	// 克莱姆法则求逆（双静态刚体退化时 K=0：约束禁用，求解跳过）
	float det = m_K11 * m_K22 - m_K12 * m_K12;
	if (det != 0.0f)
	{
		float invDet = 1.0f / det;
		m_invK11 = m_K22 * invDet;
		m_invK12 = -m_K12 * invDet;
		m_invK22 = m_K11 * invDet;
	}
	else
	{
		m_invK11 = m_invK12 = m_invK22 = 0.0f;
	}

	// 角有效质量（1-DOF 马达）：1 / (iA + iB)
	m_angularMass = (iA + iB > 0.0f) ? 1.0f / (iA + iB) : 0.0f;

	// 主约束零速度偏置（Box2D 2.4 风格，与 Revolute/Prismatic/Pulley 一致）：
	// β/dt·(pB-pA) 偏置会给锚点注入与位置误差成正比的速度，而本引擎位置修正
	// 每帧精确求解（1e-6 收敛）——剧烈外力推拉时偏置速度会在次帧造成过冲振荡
	// （RigidAxleLock 的 <1e-4 验收必挂），位置误差全部交给位置修正当帧归零
	m_bias = Vector2(0.0f, 0.0f);

	// 马达每帧冲量钳位上限：λmax = dt · τmax
	m_maxMotorImpulse = dt * m_maxMotorTorque;

	// --- 双轨冲量热启动 ---
	// 1) 2D 轮轴线冲量：维持轮子锁死在车轴悬挂点上
	Vector2 P = m_perp * m_linearImpulse.getX() + m_axis * m_linearImpulse.getY();
	m_bodyA->setVelocity(m_bodyA->getVelocity() - P * mA);
	m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - iA * Vector2::cross(m_rA, P));
	m_bodyB->setVelocity(m_bodyB->getVelocity() + P * mB);
	m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + iB * Vector2::cross(m_rB, P));

	// 2) 1D 轮毂马达扭矩冲量：维持轮子持续转动（车身承受等大反向反作用）
	if (m_enableMotor && m_angularMass > 0.0f)
	{
		m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - iA * m_motorImpulse);
		m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + iB * m_motorImpulse);
	}
}

void WheelJoint::solveVelocityConstraints()
{
	Vector2 vA = m_bodyA->getVelocity();
	float wA = m_bodyA->getAngularVelocity();
	Vector2 vB = m_bodyB->getVelocity();
	float wB = m_bodyB->getAngularVelocity();
	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// ---- 第一步：1-DOF 轮毂驱动马达（双向对称饱和）----
	// 角速度误差 Cdot = (ωB - ωA) - ω_motor，冲量 Δλ = -m_ang·Cdot 把相对转速
	// 精确推到目标值；车身承受严格等大反向的反作用冲量（急加速"抬头"、
	// 急刹"点头"力学的来源）
	if (m_enableMotor && m_angularMass > 0.0f)
	{
		float Cdot = wB - wA - m_motorSpeed;
		float impulse = -m_angularMass * Cdot;
		float oldImpulse = m_motorImpulse;
		// 扭矩钳位：累积马达冲量截断在 [-dt·τmax, +dt·τmax]
		m_motorImpulse = std::max(-m_maxMotorImpulse, std::min(m_maxMotorImpulse, m_motorImpulse + impulse));
		impulse = m_motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// ---- 第二步：2-DOF 刚性轮轴约束 ----
	// 锚点相对速度在正交坐标系 (n, s) 下的投影，解 K·λ = -(Cdot + bias)
	Vector2 Cdot(m_perp.dot(vB - vA) + m_nB * wB - m_nA * wA,
		m_axis.dot(vB - vA) + m_sB * wB - m_sA * wA);
	Vector2 B = -(Cdot + m_bias);
	Vector2 lambda(m_invK11 * B.getX() + m_invK12 * B.getY(),
		m_invK12 * B.getX() + m_invK22 * B.getY());
	m_linearImpulse += lambda;

	// 世界合力冲量 P = λn·n + λs·s
	Vector2 P = m_perp * lambda.getX() + m_axis * lambda.getY();
	vA -= P * mA;
	wA -= iA * Vector2::cross(m_rA, P);
	vB += P * mB;
	wB += iB * Vector2::cross(m_rB, P);

	m_bodyA->setVelocity(vA);
	m_bodyA->setAngularVelocity(wA);
	m_bodyB->setVelocity(vB);
	m_bodyB->setAngularVelocity(wB);
}

bool WheelJoint::solvePositionConstraints()
{
	float aA = m_bodyA->getRotation();
	float aB = m_bodyB->getRotation();
	m_rA = m_localAnchorA.rotate(aA);
	m_rB = m_localAnchorB.rotate(aB);
	Vector2 pA = m_bodyA->getPosition() + m_rA;
	Vector2 pB = m_bodyB->getPosition() + m_rB;
	Vector2 C = pB - pA;

	// 位置迭代每轮都用最新旋转重算世界轴与力矩臂（缓存会过期，Prismatic 同款教训）
	m_axis = m_localXAxisA.rotate(aA);
	m_perp = m_localYAxisA.rotate(aA);

	// 收敛判据（1e-6 门限，与 Revolute/Prismatic 锚点修正同款）：
	// 不能用 5mm 的 LINEAR_SLOP——每帧重力积分下沉 g·dt² ≈ 2.7mm 永远够不到
	// 修正门槛，轮轴会以毫米级幅度振荡（RigidAxleLock 要求 < 1e-4）
	if (C.length() < 1e-6f)
		return true;

	float mA = m_bodyA->getInvMass(), mB = m_bodyB->getInvMass();
	float iA = m_bodyA->getInvInertia(), iB = m_bodyB->getInvInertia();

	// 新鲜力矩臂投影 + 瞬时 K 矩阵 + 克莱姆法则逆（双静态刚体退化时跳过）
	m_sA = Vector2::cross(m_rA, m_axis);
	m_sB = Vector2::cross(m_rB, m_axis);
	m_nA = Vector2::cross(m_rA, m_perp);
	m_nB = Vector2::cross(m_rB, m_perp);
	float K11 = mA + mB + iA * m_nA * m_nA + iB * m_nB * m_nB;
	float K12 = iA * m_nA * m_sA + iB * m_nB * m_sB;
	float K22 = mA + mB + iA * m_sA * m_sA + iB * m_sB * m_sB;
	float det = K11 * K22 - K12 * K12;
	if (det == 0.0f)
		return true;

	// 误差投影到正交坐标系 (n, s)，解 K·λ = -C_basis 得"质量×位移"量纲冲量
	Vector2 Cb(m_perp.dot(C), m_axis.dot(C));
	float invDet = 1.0f / det;
	Vector2 lambda(-(K22 * Cb.getX() - K12 * Cb.getY()) * invDet,
		-(K11 * Cb.getY() - K12 * Cb.getX()) * invDet);

	// 冲量量纲 → 乘 invMass/invInertia 才是位移（静态刚体自动原地不动）；
	// setPositionQuiet/setRotationQuiet 内部会刷新 AABB
	Vector2 P = m_perp * lambda.getX() + m_axis * lambda.getY();
	m_bodyA->setPositionQuiet(m_bodyA->getPosition() - P * mA);
	m_bodyA->setRotationQuiet(aA - iA * Vector2::cross(m_rA, P));
	m_bodyB->setPositionQuiet(m_bodyB->getPosition() + P * mB);
	m_bodyB->setRotationQuiet(aB + iB * Vector2::cross(m_rB, P));

	return false;
}

Vector2 WheelJoint::getAnchorA() const
{
	return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 WheelJoint::getAnchorB() const
{
	return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}
