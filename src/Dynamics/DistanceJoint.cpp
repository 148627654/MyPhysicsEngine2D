#include "DistanceJoint.h"

DistanceJoint::DistanceJoint(const DistanceJointDef* def)
    : Joint(def),
    m_localAnchorA(def->localAnchorA),
    m_localAnchorB(def->localAnchorB),
    m_length(def->length),
    m_warmStartEnabled(def->enableWarmStart),
    m_frequencyHz(def->frequencyHz),
    m_dampingRatio(def->dampingRatio),
    m_impulse(0.0f),
    m_gamma(0.0f),
    m_bias(0.0f),
    m_firstResidual(0.0f),
    m_springSolved(false) {
}

void DistanceJoint::initVelocityConstraints(float dt)
{
    Body* a = m_bodyA;
    Body* b = m_bodyB;

    // 1. 锚点相对质心的世界向量
    m_rA = m_localAnchorA.rotate(a->getRotation());
    m_rB = m_localAnchorB.rotate(b->getRotation());

    // 2. 当前杆方向 A -> B
    Vector2 pA = a->getPosition() + m_rA;
    Vector2 pB = b->getPosition() + m_rB;
    Vector2 d = pB - pA;
    float dist = d.length();
    m_u = (dist > 1e-6f) ? d * (1.0f / dist) : Vector2(0.0f, 1.0f);

    // 3. 有效质量: k = invMassA + invMassB + invIA·(rA×u)² + invIB·(rB×u)²
    float invMassA = a->getInvMass();
    float invMassB = b->getInvMass();
    float invIA = a->getInvInertia();
    float invIB = b->getInvInertia();
    float rnA = Vector2::cross(m_rA, m_u);
    float rnB = Vector2::cross(m_rB, m_u);
    float k = invMassA + invMassB + invIA * rnA * rnA + invIB * rnB * rnB;
    m_mass = (k > 0.0f) ? 1.0f / k : 0.0f;

    // 3.5 软弹簧参数计算
    //   omega = 2π·f,  k_spring = mass·omega²
    if (m_frequencyHz > 0.0f) {
        float omega = 2.0f * Settings::PAI * m_frequencyHz;
        float C = dist - m_length;
        float h = dt;

        if (m_dampingRatio <= 0.0f) {
            // --- 无阻尼：辛欧拉弹簧（保能量，无长期漂移）---
            // 冲量只反馈位置误差：λ = -m·x·(C/h)，x = 4·sin²(ωh/2)
            // 配合引擎的辛欧拉积分（先速度后位置），离散映射行列式恰为 1：
            // 能量有界振荡（影子哈密顿量守恒），振幅绝不衰减也不发散
            // 【修复】x = 4·sin²(ωh/2)：先把 sin 算出来再平方乘 4，
            // 之前写成 x = 4·sin(...) 再 x *= x，把 4 也平方了（16sin²，质量虚大 4 倍）
            float s = std::sin(omega * h * 0.5f);
            float x = 4.0f * s * s;
            m_mass = m_mass * x;
            m_bias = C / h;
            m_gamma = 0.0f;
        }
        else {
            // --- 有阻尼：隐式欧拉（Box2D γ 方案）---
            //   gamma = 1 / (h·(d + h·k_spring))  ← 隐式项，任意刚度无条件稳定（L 稳定）
            //   bias  = C · h · k_spring · gamma
            float d = 2.0f * m_mass * m_dampingRatio * omega;
            float kSpring = m_mass * omega * omega;
            m_gamma = h * (d + h * kSpring);
            m_gamma = (m_gamma != 0.0f) ? 1.0f / m_gamma : 0.0f;
            m_bias = C * h * kSpring * m_gamma;

            // 增广有效质量: 1 / (1/mass + gamma)
            m_mass = 1.0f / (1.0f / m_mass + m_gamma);
        }
    }
    else {
        m_gamma = 0.0f;
        m_bias = 0.0f;
    }

    // 4. 热启动：把上一帧的累计冲量作为"初始猜测"立即施加
    // 无阻尼辛弹簧不做热启动：冲量状态无反馈会随机漂移，破坏保能量性质
    bool isSymplecticSpring = (m_frequencyHz > 0.0f && m_dampingRatio <= 0.0f);
    if (m_warmStartEnabled && m_impulse != 0.0f && !isSymplecticSpring) {
        Vector2 P = m_u * m_impulse;
        a->applyImpulse(-P, m_rA);
        b->applyImpulse(P, m_rB);
    }

    // 辛弹簧每帧只允许施加一次冲量（重复施加会累积过冲）
    m_springSolved = false;

    // 5. 诊断：热启动后、首轮速度迭代前的约束残差 Cdot
    {
        Vector2 vA = a->getVelocity() + Vector2::cross(a->getAngularVelocity(), m_rA);
        Vector2 vB = b->getVelocity() + Vector2::cross(b->getAngularVelocity(), m_rB);
        m_firstResidual = m_u.dot(vB - vA);
    }
}

void DistanceJoint::solveVelocityConstraints()
{
    Body* a = m_bodyA;
    Body* b = m_bodyB;

    // 相对法向速度（杆长变化率）
    Vector2 vA = a->getVelocity() + Vector2::cross(a->getAngularVelocity(), m_rA);
    Vector2 vB = b->getVelocity() + Vector2::cross(b->getAngularVelocity(), m_rB);
    float Cdot = m_u.dot(vB - vA);

    // 刚性杆：把 Cdot 精确压到 0（单约束一次迭代即可收敛）
    // 有阻尼软弹簧：Cdot + bias + gamma·impulse —— 隐式欧拉，绝对数值稳定
    // 无阻尼辛弹簧：只反馈位置误差，每帧恰好一次
    float lambda;
    if (m_frequencyHz > 0.0f && m_dampingRatio <= 0.0f) {
        if (m_springSolved) {
            return; // 本帧已施加过冲量，直接跳过（避免 8 次迭代重复累积）
        }
        m_springSolved = true;
        lambda = -m_mass * m_bias;
        m_impulse = lambda;
    }
    else if (m_frequencyHz > 0.0f) {
        lambda = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
        m_impulse += lambda;
    }
    else {
        lambda = -m_mass * Cdot;
        m_impulse += lambda;
    }

    Vector2 P = m_u * lambda;
    a->applyImpulse(-P, m_rA);
    b->applyImpulse(P, m_rB);
}

bool DistanceJoint::solvePositionConstraints()
{
    // 软弹簧不参与位置修正：位置误差已通过速度偏置（bias）交给弹簧动力学处理
    if (m_frequencyHz > 0.0f) {
        return true;
    }

    Body* a = m_bodyA;
    Body* b = m_bodyB;

    Vector2 pA = a->getPosition() + m_rA;
    Vector2 pB = b->getPosition() + m_rB;
    Vector2 d = pB - pA;
    float dist = d.length();
    if (dist < 1e-6f) {
        return true;
    }

    Vector2 u = d * (1.0f / dist);
    float C = dist - m_length; // 杆长误差

    // 单步修正钳制，防止极端穿透时位置修正爆炸
    C = std::max(-0.2f, std::min(0.2f, C));

    // 位置冲量：A 沿 +u 移动、B 沿 -u 移动，把 |C| 收缩到 0
    Vector2 P = u * (-m_mass * C);

    a->setPositionQuiet(a->getPosition() - P * a->getInvMass());
    b->setPositionQuiet(b->getPosition() + P * b->getInvMass());

    float ra = Vector2::cross(m_rA, P);
    float rb = Vector2::cross(m_rB, P);
    // 【修复】B 的角向修正符号之前是 -（与 Box2D 相反）：杆超长时 B 朝错误方向
    // 旋转，越修越长，偏心锚点的杆（吊桥/吊杆）会持续自旋泵能。
    // 正确：A 沿 -invIA·(rA×P)，B 沿 +invIB·(rB×P)
    a->setRotationQuiet(a->getRotation() - a->getInvInertia() * ra);
    b->setRotationQuiet(b->getRotation() + b->getInvInertia() * rb);

    return std::abs(C) < 0.005f; // 线性容差
}

Vector2 DistanceJoint::getAnchorA() const
{
    return m_bodyA->getPosition() + m_localAnchorA.rotate(m_bodyA->getRotation());
}

Vector2 DistanceJoint::getAnchorB() const
{
    return m_bodyB->getPosition() + m_localAnchorB.rotate(m_bodyB->getRotation());
}
