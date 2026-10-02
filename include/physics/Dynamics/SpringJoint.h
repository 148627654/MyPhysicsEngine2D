#pragma once
#include "Joint.h"

struct SpringJointDef:public JointDef
{
    Vector2 localAnchorA;          // BodyA 上的局部锚点
    Vector2 localAnchorB;          // BodyB 上的局部锚点
    float length = 0.0f;           // 目标杆长
    float frequencyHz = 2.0f;      // 振动频率（默认 2 Hz 软弹簧）。
    float dampingRatio = 0.7f;     // 阻尼比（默认 0.7 舒适避震）。
	SpringJointDef(){type = JointType::Spring;}
};

class SpringJoint : public Joint
{
public:
    SpringJoint(const SpringJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override { return true; } // 软约束允许形变，无需刚性位置投影

    Vector2 getAnchorA() const override;
    Vector2 getAnchorB() const override;

    float getFrequency() const { return m_frequencyHz; }
    void setFrequency(float hz) { m_frequencyHz = hz; }

    float getDampingRatio() const { return m_dampingRatio; }
    void setDampingRatio(float ratio) { m_dampingRatio = ratio; }
private:
    float m_length;
    float m_frequencyHz;
    float m_dampingRatio;
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;

    // --- 解算器缓存 ---
    float m_impulse = 0.0f;
    Vector2 m_u;
    Vector2 m_rA, m_rB;
    float m_crAu = 0.0f;
    float m_crBu = 0.0f;
    float m_mass = 0.0f;       // 软化有效质量 (m_soft)
    float m_bias = 0.0f;       // 软化速度偏置
    float m_gamma = 0.0f;      // 柔度参数 (Softness)
};