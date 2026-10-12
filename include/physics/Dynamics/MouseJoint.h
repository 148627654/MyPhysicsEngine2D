#pragma once
#include "Joint.h"

// 鼠标关节定义：2-DOF 隐式软弹簧阻尼器 + 库仑圆盘最大拉力截断
// （Box2D 鼠标关节同款拓扑：bodyA 一般为静态地面刚体，bodyB 为被拖拽刚体）
struct MouseJointDef : public JointDef
{
    MouseJointDef() { type = JointType::Mouse; }
    Vector2 target;              // 初始鼠标世界目标点
    Vector2 localAnchorB;        // 抓取点在 B 上的局部坐标（装配时 = 光标点相对 B 质心的局部位置）
    float maxForce = 0.0f;       // 最大允许拖拽拉力 (N)，推荐 1000 × bodyB 质量
    float frequencyHz = 5.0f;    // 弹簧追踪频率 (Hz)：8~10 跟手坚实，2 果冻拖拽感
    float dampingRatio = 0.7f;   // 阻尼比（0.7 = 临界抗抖，无高频抖动）
};

class MouseJoint : public Joint
{
public:
    MouseJoint(const MouseJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override;

    Vector2 getAnchorA() const override; // 鼠标关节的"锚点 A"即动态目标点
    Vector2 getAnchorB() const override; // 抓取点世界坐标

    // --- 运行时动态目标点 API（外部鼠标移动事件调用）---
    void setTarget(const Vector2& target) { m_target = target; }
    Vector2 getTarget() const { return m_target; }
    void setMaxForce(float force) { m_maxForce = force; }
    float getMaxForce() const { return m_maxForce; }
    // 本帧实际输出的 2D 拖拽拉力 (N)（已被向量限幅保证模长 ≤ maxForce）
    Vector2 getForce(float dt) const {
        return (dt > 0.0f) ? m_impulse / dt : Vector2(0.0f, 0.0f);
    }
    Vector2 getImpulse() const { return m_impulse; }

private:
    Vector2 m_localAnchorB;       // 抓取点在 B 上的局部坐标
    Vector2 m_target;             // 动态鼠标世界目标点
    float m_maxForce = 0.0f;
    float m_frequencyHz = 0.0f;
    float m_dampingRatio = 0.0f;

    // 弹簧软约束参数（隐式欧拉离散，Box2D 同款）
    float m_gamma = 0.0f;   // 柔度反作用系数 γ = 1/(h·(d + h·k))
    float m_beta = 0.0f;    // 偏置系数 β = h·k·γ（量纲 s⁻¹）
    float m_dt = 0.0f;      // 本帧步长缓存（限幅上限 λmax = dt·Fmax 用）
    Vector2 m_bias = { 0.0f, 0.0f };   // 2D 弹簧速度偏置 v_bias = β·(pB - target)

    // --- 解算缓存 ---
    Vector2 m_rB;                         // 世界力矩臂
    Vector2 m_impulse = { 0.0f, 0.0f };   // 2D 累计拉力冲量（热启动 + 圆盘限幅）
    // K_soft = K + γI 的克莱姆法则逆（对称矩阵，存 3 个标量）
    float m_softMass11 = 0.0f, m_softMass12 = 0.0f, m_softMass22 = 0.0f;
};
