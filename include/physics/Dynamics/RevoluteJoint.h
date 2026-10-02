#pragma once
#include "Joint.h"

// 限位状态机
enum class LimitState {
    Inactive,  // 自由旋转，限位不生效
    AtLower,   // 触碰下限位（只能产生使角度增大的正冲量）
    AtUpper,   // 触碰上限位（只能产生使角度减小的负冲量）
    Equal      // 上下限相等：双向等式约束，锁死角度（刚性焊接）
};

struct RevoluteJointDef :public JointDef
{
    Vector2 localAnchorA;          // BodyA 上的局部锚点
    Vector2 localAnchorB;          // BodyB 上的局部锚点
    float referenceAngle = 0.0f;   // 装配时的基准相对角度 θB0 - θA0
    RevoluteJointDef() { type = JointType::Revolute; }

    // 便捷装配：给定世界锚点，自动计算局部锚点与基准角
    void initialize(Body* bA, Body* bB, const Vector2& worldAnchor) {
        bodyA = bA;
        bodyB = bB;
        localAnchorA = (worldAnchor - bA->getPosition()).rotate(-bA->getRotation());
        localAnchorB = (worldAnchor - bB->getPosition()).rotate(-bB->getRotation());
        referenceAngle = bB->getRotation() - bA->getRotation();
    }

    // --- 角度限位配置 ---
    bool enableLimit = false;
    float lowerAngle = 0.0f;    // 最小相对角度（弧度）
    float upperAngle = 0.0f;    // 最大相对角度（弧度）

    // --- 马达配置 ---
    bool enableMotor = false;
    float motorSpeed = 0.0f;    // 目标转速 (rad/s)
    float maxMotorTorque = 0.0f;// 最大输出扭矩 (N·m)
};

class RevoluteJoint : public Joint
{
public:
    RevoluteJoint(const RevoluteJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override;

    Vector2 getAnchorA() const override;
    Vector2 getAnchorB() const override;

    Vector2 getLocalAnchorA() const { return m_localAnchorA; }
    Vector2 getLocalAnchorB() const { return m_localAnchorB; }

    // --- 运行时控制 API（马达 / 限位）---
    void setMotorSpeed(float speed) { m_motorSpeed = speed; }
    void setMaxMotorTorque(float torque) { m_maxMotorTorque = torque; }
    void enableMotor(bool flag) { m_enableMotor = flag; if (!flag) m_motorImpulse = 0.0f; }
    void enableLimit(bool flag) { m_enableLimit = flag; if (!flag) m_limitImpulse = 0.0f; }
    // 当前相对角度 θB - θA - θref
    float getJointAngle() const {
        return m_bodyB->getRotation() - m_bodyA->getRotation() - m_referenceAngle;
    }
    // 本帧马达输出的等效扭矩 (N·m)
    float getMotorTorque(float dt) const {
        return (dt > 0.0f) ? m_motorImpulse / dt : 0.0f;
    }
    // 当前限位状态（诊断/游戏逻辑用）
    LimitState getLimitState() const { return m_limitState; }

private:
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;

    // --- 约束解算缓存（2-DOF 销钉）---
    Vector2 m_impulse = { 0.0f, 0.0f }; // 2D 累计约束冲量 (用于 Warm Starting)
    Vector2 m_rA, m_rB;               // 世界坐标力矩臂

    // 2x2 有效质量矩阵元素 (K 矩阵是对称矩阵，只需存 3 个标量)
    float m_K11 = 0.0f;
    float m_K12 = 0.0f;
    float m_K22 = 0.0f;

    Vector2 m_bias = { 0.0f, 0.0f };    // 2D 稳定化速度偏置

    // --- 马达配置与状态 ---
    bool m_enableMotor = false;
    float m_motorSpeed = 0.0f;
    float m_maxMotorTorque = 0.0f;

    // --- 限位配置与状态 ---
    bool m_enableLimit = false;
    float m_lowerAngle = 0.0f;
    float m_upperAngle = 0.0f;
    float m_referenceAngle = 0.0f;
    LimitState m_limitState = LimitState::Inactive;

    // --- 角自由度（1-DOF）解算缓存 ---
    // 马达与限位冲量必须独立记账：马达在 ±τmax·dt 双向饱和，限位单向截断，
    // 混在一起热启动会互相污染（马达冲破限位）
    float m_motorImpulse = 0.0f;
    float m_limitImpulse = 0.0f;
    float m_angularMass = 0.0f;  // 1 / (iA + iB)
    float m_limitBias = 0.0f;    // 限位 Baumgarte 角度速度偏置
    float m_maxMotorImpulse = 0.0f; // 每帧马达冲量钳位上限 = dt · τmax
};