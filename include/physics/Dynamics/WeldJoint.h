#pragma once
#include "Joint.h"

struct Vector3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    Vector3() = default;
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct WeldJointDef : public JointDef
{
	Vector2 localAnchorA;          // BodyA 上的局部锚点
	Vector2 localAnchorB;          // BodyB 上的局部锚点
	float referenceAngle = 0.0f;   // 初始角度差（BodyB 相对于 BodyA 的角度）
	WeldJointDef() { type = JointType::Weld; }
};

class WeldJoint : public Joint {
public:
    WeldJoint(const WeldJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override;

    Vector2 getAnchorA() const override;
    Vector2 getAnchorB() const override;

private:
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;
    float m_referenceAngle;

    // --- 解算缓存 ---
    Vector3 m_impulse = { 0.0f, 0.0f, 0.0f }; // 3D 累积冲量 [λx, λy, λ_torque]
    Vector2 m_rA, m_rB;                    // 世界力矩臂

    // 3x3 对称有效质量矩阵元素
    float m_K11, m_K12, m_K13;
    float m_K22, m_K23;
    float m_K33;

    Vector3 m_bias;                        // 3D 稳定化偏置速度
};