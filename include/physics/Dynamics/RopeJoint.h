#pragma once

#include "Joint.h"
#include "../Common/Setting.h"
struct RopeJointDef : public JointDef
{
	Vector2 localAnchorA;          // BodyA 上的局部锚点
	Vector2 localAnchorB;          // BodyB 上的局部锚点
	float maxLength = 0.0f;        // 最大绳长（世界坐标系）
	RopeJointDef() { type = JointType::Rope; }
};

class RopeJoint : public Joint
{
public:
	RopeJoint(const RopeJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override;

    Vector2 getAnchorA() const override;
    Vector2 getAnchorB() const override;

    // --- 运行时控制与诊断 API ---
    float getMaxLength() const { return m_maxLength; }
    void setMaxLength(float maxLen) { m_maxLength = maxLen; }
    LimitState getState() const { return m_state; }
    // 当前累计拉力冲量（约定 λ ≤ 0：负值=拉力，0=松弛无张力）
    float getImpulse() const { return m_impulse; }

private:
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;
    float m_maxLength;

    // --- 解算缓存 ---
    float m_impulse = 0.0f; // 单向累积拉力冲量 (λ <= 0)
    Vector2 m_u;            // 单位拉力方向 (由 A 指向 B)
    Vector2 m_rA, m_rB;     // 世界力矩臂
    float m_crAu = 0.0f;    // rA x u
    float m_crBu = 0.0f;    // rB x u
    float m_mass = 0.0f;    // 有效质量 1 / K
    float m_bias = 0.0f;    // 绷紧时的速度偏置

    LimitState m_state = LimitState::Inactive; // 当前绳索状态 (Inactive: 松弛, AtUpper: 绷直)
};
