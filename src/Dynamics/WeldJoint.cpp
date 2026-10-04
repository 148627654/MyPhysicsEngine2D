#include "WeldJoint.h"
#include "../common/Setting.h"
#include <cmath>

WeldJoint::WeldJoint(const WeldJointDef* def)
	:Joint(def),
	m_localAnchorA(def->localAnchorA),
	m_localAnchorB(def->localAnchorB),
	m_referenceAngle(def->referenceAngle) {}

void WeldJoint::initVelocityConstraints(float dt)
{
    m_rA = m_localAnchorA.rotate(m_bodyA->getRotation());
    m_rB = m_localAnchorB.rotate(m_bodyB->getRotation());

    Vector2 pA = m_bodyA->getPosition() + m_rA;
    Vector2 pB = m_bodyB->getPosition() + m_rB;

    // 3*3 矩阵（推导完全满分！）
    m_K11 = m_bodyA->getInvMass() + m_bodyB->getInvMass() + m_bodyA->getInvInertia() * m_rA.y * m_rA.y + m_bodyB->getInvInertia() * m_rB.y * m_rB.y;
    m_K12 = -m_bodyA->getInvInertia() * m_rA.x * m_rA.y - m_bodyB->getInvInertia() * m_rB.x * m_rB.y;
    m_K22 = m_bodyA->getInvMass() + m_bodyB->getInvMass() + m_bodyA->getInvInertia() * m_rA.x * m_rA.x + m_bodyB->getInvInertia() * m_rB.x * m_rB.x;

    m_K33 = m_bodyA->getInvInertia() + m_bodyB->getInvInertia();

    m_K23 = m_bodyA->getInvInertia() * m_rA.x + m_bodyB->getInvInertia() * m_rB.x;
    m_K13 = -m_bodyA->getInvInertia() * m_rA.y - m_bodyB->getInvInertia() * m_rB.y;

    // 计算三维速度偏置
    Vector2 C1 = pB - pA;
    float C_angle = m_bodyB->getRotation() - m_bodyA->getRotation() - m_referenceAngle;
    float beta = 0.2f;
    float invDt = (dt > 0.0f) ? (1.0f / dt) : 0.0f;

    // 直接赋值给成员变量 m_bias
    m_bias.x = beta * C1.x * invDt;
    m_bias.y = beta * C1.y * invDt;
    m_bias.z = beta * C_angle * invDt;

    // 热启动：线冲量 + 纯角度锁定冲量
    Vector2 P = Vector2(m_impulse.x, m_impulse.y);
    float torqueImpulse = m_impulse.z;

    m_bodyA->applyImpulse(-P, m_rA);
    m_bodyB->applyImpulse(P, m_rB);

    m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - m_bodyA->getInvInertia() * torqueImpulse);
    m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + m_bodyB->getInvInertia() * torqueImpulse);
}

void WeldJoint::solveVelocityConstraints()
{
    // 获取最新的线速度和角速度
	Vector2 vA = m_bodyA->getVelocity();
    float angle_A = m_bodyA->getAngularVelocity();
	Vector2 vB = m_bodyB->getVelocity();
	float angle_B = m_bodyB->getAngularVelocity();
	// 组装三维速度向量
    // 1. 分别计算两锚点的旋转切向速度 (w x r = [-w*ry, w*rx])
    Vector2 wA(-angle_A * m_rA.y, angle_A * m_rA.x);
    Vector2 wB(-angle_B * m_rB.y, angle_B * m_rB.x);

    // 2. 组装三维相对速度向量 C_dot (清晰易懂！)
    Vector2 linearCdot = (vB + wB) - (vA + wA);
    Vector3 C_dot = {
        linearCdot.x,
        linearCdot.y,
        angle_B - angle_A
    };
	Vector3 B = {
		-C_dot.x - m_bias.x,
		-C_dot.y - m_bias.y,
		-C_dot.z - m_bias.z
	};
	// 求解3*3线性方程组 K * λ = B，得到冲量 λ
    // c1,c2,c3三个列向量
	Vector3 c1(m_K11, m_K12, m_K13);
	Vector3 c2(m_K12, m_K22, m_K23);
	Vector3 c3(m_K13, m_K23, m_K33);

	float det = c1.x * (c2.y * c3.z - c2.z * c3.y) -
		c1.y * (c2.x * c3.z - c2.z * c3.x) +
		c1.z * (c2.x * c3.y - c2.y * c3.x);
    if (det != 0)
    {
        float invDet = 1.0f / det;
        Vector3 lambda;
        lambda.x = invDet * (B.x * (c2.y * c3.z - c2.z * c3.y) -
            B.y * (c2.x * c3.z - c2.z * c3.x) +
            B.z * (c2.x * c3.y - c2.y * c3.x));
        lambda.y = invDet * (-B.x * (c1.y * c3.z - c1.z * c3.y) +
            B.y * (c1.x * c3.z - c1.z * c3.x) -
            B.z * (c1.x * c3.y - c1.y * c3.x));
        lambda.z = invDet * (B.x * (c1.y * c2.z - c1.z * c2.y) -
            B.y * (c1.x * c2.z - c1.z * c2.x) +
            B.z * (c1.x * c2.y - c1.y * c2.x));
        // 累积冲量
        m_impulse.x += lambda.x;
        m_impulse.y += lambda.y;
        m_impulse.z += lambda.z;
        // 应用冲量
        Vector2 P(lambda.x, lambda.y);
        m_bodyA->applyImpulse(-P, m_rA);
        m_bodyB->applyImpulse(P, m_rB);
        m_bodyA->setAngularVelocity(m_bodyA->getAngularVelocity() - m_bodyA->getInvInertia() * lambda.z);
        m_bodyB->setAngularVelocity(m_bodyB->getAngularVelocity() + m_bodyB->getInvInertia() * lambda.z);
    }
}
bool WeldJoint::solvePositionConstraints()
{
    Vector2 pA = m_bodyA->getPosition();
    float aA = m_bodyA->getRotation();
    Vector2 pB = m_bodyB->getPosition();
    float aB = m_bodyB->getRotation();

    // 位置迭代每轮都用最新旋转重算力矩臂（上一帧缓存的 m_rA/m_rB 已过期）
    m_rA = m_localAnchorA.rotate(aA);
    m_rB = m_localAnchorB.rotate(aB);

    Vector2 C1 = (pB + m_rB) - (pA + m_rA);
    float C_angle = aB - aA - m_referenceAngle;
    Vector3 C = { C1.x, C1.y, C_angle };
    if (C1.length() < Settings::LINEAR_SLOP && std::abs(C_angle) <= Settings::ANGULAR_SLOP)
        return true;
    // 3*3 K 矩阵（与速度求解同构，使用最新的旋转角度计算）
    m_K11 = m_bodyA->getInvMass() + m_bodyB->getInvMass() + m_bodyA->getInvInertia() * m_rA.y * m_rA.y + m_bodyB->getInvInertia() * m_rB.y * m_rB.y;
    m_K12 = -m_bodyA->getInvInertia() * m_rA.x * m_rA.y - m_bodyB->getInvInertia() * m_rB.x * m_rB.y;
    m_K22 = m_bodyA->getInvMass() + m_bodyB->getInvMass() + m_bodyA->getInvInertia() * m_rA.x * m_rA.x + m_bodyB->getInvInertia() * m_rB.x * m_rB.x;

    m_K33 = m_bodyA->getInvInertia() + m_bodyB->getInvInertia();

    m_K23 = m_bodyA->getInvInertia() * m_rA.x + m_bodyB->getInvInertia() * m_rB.x;
    m_K13 = -m_bodyA->getInvInertia() * m_rA.y - m_bodyB->getInvInertia() * m_rB.y;
    // K * P = -C
    Vector3 B = { -C.x, -C.y, -C.z };
    float det = m_K11 * (m_K22 * m_K33 - m_K23 * m_K23) -
        m_K12 * (m_K12 * m_K33 - m_K13 * m_K23) +
        m_K13 * (m_K12 * m_K23 - m_K13 * m_K22);
    if (det != 0)
    {
        float invDet = 1.0f / det;
        Vector3 P;
        P.x = invDet * (B.x * (m_K22 * m_K33 - m_K23 * m_K23) -
            B.y * (m_K12 * m_K33 - m_K13 * m_K23) +
            B.z * (m_K12 * m_K23 - m_K13 * m_K22));
        P.y = invDet * (-B.x * (m_K12 * m_K33 - m_K13 * m_K23) +
            B.y * (m_K11 * m_K33 - m_K13 * m_K13) -
            B.z * (m_K11 * m_K23 - m_K12 * m_K13));
        P.z = invDet * (B.x * (m_K12 * m_K23 - m_K13 * m_K22) -
            B.y * (m_K11 * m_K23 - m_K12 * m_K13) +
            B.z * (m_K11 * m_K22 - m_K12 * m_K12));

        // 位置修正：K⁻¹·(-C) 是"质量×位移"量纲的冲量，乘 invMass/invInertia 才是位移
        // （静态刚体 invMass=0 自动原地不动）
        Vector2 linearImpulse(P.x, P.y);
        float angularImpulse = P.z;
        m_bodyA->setPositionQuiet(m_bodyA->getPosition() - linearImpulse * m_bodyA->getInvMass());
        m_bodyA->setRotationQuiet(aA - angularImpulse * m_bodyA->getInvInertia());
        m_bodyB->setPositionQuiet(m_bodyB->getPosition() + linearImpulse * m_bodyB->getInvMass());
        m_bodyB->setRotationQuiet(aB + angularImpulse * m_bodyB->getInvInertia());

        // 修正后重算误差，作为收敛判据
        C1 = (m_bodyB->getPosition() + m_rB) - (m_bodyA->getPosition() + m_rA);
        C_angle = m_bodyB->getRotation() - m_bodyA->getRotation() - m_referenceAngle;
        return C1.length() < Settings::LINEAR_SLOP && std::abs(C_angle) <= Settings::ANGULAR_SLOP;
    }

    return false;
}

Vector2 WeldJoint::getAnchorA() const
{
	return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 WeldJoint::getAnchorB() const
{
	return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}
