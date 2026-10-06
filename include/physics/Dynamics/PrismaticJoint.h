#pragma once
#include "Joint.h"
#include "../Common/Vector2.h"

struct PrismaticJointDef:public JointDef
{
	PrismaticJointDef() { type = JointType::Prismatic; }
    Vector2 localAnchorA ;
    Vector2 localAnchorB ;
    Vector2 localAxisA ; // A 本地坐标系下的滑轨方向
    float referenceAngle = 0.0f;

    bool enableLimit = false;
    float lowerTranslation = 0.0f;
    float upperTranslation = 0.0f;

    // --- 线性马达配置 ---
    bool enableMotor = false;
    float motorSpeed = 0.0f;    // 目标滑动速度 (m/s)
    float maxMotorForce = 0.0f; // 最大输出推力 (N)

    // 便捷装配：给定世界锚点，自动计算局部锚点与基准角（与 RevoluteJointDef 同款）
    void initialize(Body* bA, Body* bB, const Vector2& worldAnchor) {
        bodyA = bA;
        bodyB = bB;
        localAnchorA = (worldAnchor - bA->getPosition()).rotate(-bA->getRotation());
        localAnchorB = (worldAnchor - bB->getPosition()).rotate(-bB->getRotation());
        referenceAngle = bB->getRotation() - bA->getRotation();
    }
};

class PrismaticJoint : public Joint
{
public:
	PrismaticJoint(const PrismaticJointDef* def);
	void initVelocityConstraints(float dt) override;
	void solveVelocityConstraints() override;
	bool solvePositionConstraints() override;
	Vector2 getAnchorA() const override;
	Vector2 getAnchorB() const override;

	// 当前沿轴位移 t·(锚点B - 锚点A)（诊断/游戏逻辑用）
	float getJointTranslation() const {
		Vector2 d = getAnchorB() - getAnchorA();
		return m_localXAxisA.rotate(m_bodyA->getRotation()).dot(d);
	}
	// 当前限位状态（诊断/游戏逻辑用）
	LimitState getLimitState() const { return m_limitState; }

	// --- 运行时控制 API（马达）---
	void setMotorSpeed(float speed) { m_motorSpeed = speed; }
	void setMaxMotorForce(float force) { m_maxMotorForce = force; }
	void enableMotor(bool flag) { m_enableMotor = flag; if (!flag) m_motorImpulse = 0.0f; }
	// 本帧马达输出的等效推力 (N)
	float getMotorForce(float dt) const {
		return (dt > 0.0f) ? m_motorImpulse / dt : 0.0f;
	}
private:
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;
    Vector2 m_localXAxisA;
    Vector2 m_localYAxisA; // 局部垂向轴
    float m_referenceAngle;

    // 限位配置与状态
    bool m_enableLimit;
    float m_lowerTranslation;
    float m_upperTranslation;
    LimitState m_limitState = LimitState::Inactive;

    // 马达配置与状态（与限位冲量绝对隔离：马达双向对称饱和、限位单向截断）
    bool m_enableMotor = false;
    float m_motorSpeed = 0.0f;
    float m_maxMotorForce = 0.0f;
    float m_motorImpulse = 0.0f;      // 独立缓存马达冲量
    float m_maxMotorImpulse = 0.0f;   // 每帧马达冲量钳位上限 = dt · Fmax

    // --- 解算缓存 ---
    Vector2 m_impulse ;               // 2-DOF 基础冲量 [λ_perp, λ_angular]
    float m_limitImpulse = 0.0f;      // 1-DOF 轴向限位冲量

    Vector2 m_axis, m_perp;           // 世界滑轨轴 t 与 垂轴 n
    float m_s1, m_s2;                 // 垂向力矩臂投影 (sA x n, rB x n)
    float m_a1, m_a2;                 // 轴向力矩臂投影 (sA x t, rB x t)

    // 2x2 基础有效质量矩阵分量 (对称)
    float m_K11, m_K12, m_K22;
    float m_axialMass = 0.0f;         // 轴向限位有效质量

    Vector2 m_bias;
    float m_limitBias = 0.0f;
};