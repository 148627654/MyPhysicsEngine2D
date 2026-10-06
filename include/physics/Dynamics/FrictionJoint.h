#pragma once
#include "Joint.h"

struct FrictionJointDef : public JointDef
{
	FrictionJointDef() { type = JointType::Friction; }
    Vector2 localAnchorA;          // BodyA 上的局部锚点
    Vector2 localAnchorB;          // BodyB 上的局部锚点
    float maxForce = 0.0f;         // 最大线摩擦阻尼力 (N)
    float maxTorque = 0.0f;        // 最大角摩擦阻尼扭矩 (N·m)

    // 便捷装配：给定世界锚点，自动计算局部锚点（与 RevoluteJointDef 同款）
    void initialize(Body* bA, Body* bB, const Vector2& worldAnchor) {
        bodyA = bA;
        bodyB = bB;
        localAnchorA = (worldAnchor - bA->getPosition()).rotate(-bA->getRotation());
        localAnchorB = (worldAnchor - bB->getPosition()).rotate(-bB->getRotation());
    }
};

/// @brief 摩擦关节：有界饱和冲量的耗散约束（引擎第一个"阻尼类"关节）
///
/// 线摩擦（2-DOF）：两锚点相对线速度归零，累计冲量做**库仑圆截断**
///   ——平面各向摩擦上限相同，必须对向量模长截断，不能分轴截断。
/// 角摩擦（1-DOF）：相对转速归零，累计扭矩冲量限制在 [-dt·τmax, +dt·τmax]。
/// 位置解算旁路：摩擦力是耗散力，绝不拉回历史位置。
class FrictionJoint : public Joint
{
public:
    FrictionJoint(const FrictionJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override;

    Vector2 getAnchorA() const override;
    Vector2 getAnchorB() const override;

    // --- 运行时控制 API ---
    void setMaxForce(float force) { m_maxForce = force; }
    void setMaxTorque(float torque) { m_maxTorque = torque; }

private:
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;

    float m_maxForce;   // 最大线摩擦阻尼力 (N)
    float m_maxTorque;  // 最大角摩擦阻尼扭矩 (N·m)

    // --- 解算缓存 ---
    Vector2 m_rA, m_rB;                    // 世界力矩臂
    float m_K11 = 0.0f, m_K12 = 0.0f, m_K22 = 0.0f; // 线 2×2 有效质量矩阵（对称）
    float m_angularMass = 0.0f;            // 角有效质量 1 / (iA + iB)
    float m_dt = 0.0f;                     // 本帧步长（冲量钳位用）

    Vector2 m_linearImpulse = { 0.0f, 0.0f }; // 2D 线摩擦累计冲量（库仑圆截断）
    float m_angularImpulse = 0.0f;            // 1D 角摩擦累计冲量（标量区间截断）
};
