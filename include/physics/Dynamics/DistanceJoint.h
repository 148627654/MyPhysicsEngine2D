#pragma once
#include "Joint.h"
#include "../Common/Vector2.h"

// 距离关节（刚性杆/弹性弹簧）定义
struct DistanceJointDef : JointDef {
    Vector2 localAnchorA;          // BodyA 上的局部锚点
    Vector2 localAnchorB;          // BodyB 上的局部锚点
    float length = 0.0f;           // 目标杆长
    bool enableWarmStart = true;   // 是否启用热启动（累积冲量缓存）

    // --- 软弹簧参数（frequencyHz > 0 时启用，隐式欧拉积分，无条件稳定）---
    float frequencyHz = 0.0f;      // 弹簧固有频率 (Hz)，0 = 纯刚性杆
    float dampingRatio = 0.0f;     // 阻尼比: 0=无阻尼, 1=临界阻尼

    DistanceJointDef() { type = JointType::Distance; }
};

// 刚性距离关节：
//   速度约束  Cdot = u·(vB + wB×rB - vA - wA×rA) -> 0  （冲量法，含热启动）
//   位置约束  C = |pB + rB - pA - rA| - L -> 0         （Baumgarte 位置修正，带单步钳制）
class DistanceJoint : public Joint {
public:
    DistanceJoint(const DistanceJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override;
    Vector2 getAnchorA() const override;
    Vector2 getAnchorB() const override;

    // 诊断接口（供测试对比热启动效果）：
    // 热启动（或未热启动）之后、首轮速度迭代之前的约束残差
    float getFirstResidual() const { return m_firstResidual; }
    // 本帧累计冲量（热启动缓存）
    float getAccumulatedImpulse() const { return m_impulse; }

private:
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;
    float m_length;
    bool m_warmStartEnabled;
    float m_frequencyHz;   // 弹簧固有频率 (Hz)，0 = 刚性
    float m_dampingRatio;  // 阻尼比

    // --- 每帧解算状态 ---
    Vector2 m_rA, m_rB;    // 锚点相对质心的向量
    Vector2 m_u;           // 单位方向 A -> B
    float m_mass;          // 有效质量 1/k（软弹簧时含 gamma 增广）
    float m_impulse;       // 累计冲量（热启动缓存）
    float m_gamma;         // 隐式欧拉阻尼项系数（软弹簧）
    float m_bias;          // 弹簧位置误差速度偏置（软弹簧）
    float m_firstResidual; // 首轮迭代前的约束残差（诊断）
    bool m_springSolved;   // 辛弹簧本帧是否已施加过冲量（每帧只允许一次）
};
