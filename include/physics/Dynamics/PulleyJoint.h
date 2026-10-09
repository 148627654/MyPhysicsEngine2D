#pragma once

#include "Joint.h"
#include "../Common/Setting.h"
struct PulleyJointDef : public JointDef {
	PulleyJointDef() {
		type = JointType::Pulley;
	}
	Vector2 groundAnchorA; ///< 世界坐标系下的滑轮 A 锚点
	Vector2 groundAnchorB; ///< 世界坐标系下的滑轮 B 锚点
	Vector2 localAnchorA;  ///< BodyA 上的锚点局部坐标
	Vector2 localAnchorB;  ///< BodyB 上的锚点局部坐标
	float lengthA = 0.0f;  ///< BodyA 到滑轮 A 的绳索长度
	float lengthB = 0.0f;  ///< BodyB 到滑轮 B 的绳索长度
	float ratio = 1.0f;    ///< 绳索传动比（BodyA 绳索长度变化量 / BodyB 绳索长度变化量）
};

class PulleyJoint : public Joint {
public:
    PulleyJoint(const PulleyJointDef* def);

    void initVelocityConstraints(float dt) override;
    void solveVelocityConstraints() override;
    bool solvePositionConstraints() override;

    Vector2 getAnchorA() const override;
    Vector2 getAnchorB() const override;
    Vector2 getGroundAnchorA() const { return m_groundAnchorA; }
    Vector2 getGroundAnchorB() const { return m_groundAnchorB; }

    float getLengthA() const;
    float getLengthB() const;
    float getRatio() const { return m_ratio; }

private:
    Vector2 m_groundAnchorA;
    Vector2 m_groundAnchorB;
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;
    float m_constant;          // 总等效绳长: lengthA + ratio * lengthB
    float m_ratio;

    // --- 解算缓存 ---
    float m_impulse = 0.0f;
    Vector2 m_uA, m_uB;        // 地面吊点指向刚体锚点的单位向量
    Vector2 m_rA, m_rB;        // 世界力矩臂
    float m_crAu = 0.0f;       // rA x uA
    float m_crBu = 0.0f;       // rB x uB
    float m_mass = 0.0f;       // 1 / K (含 ratio^2 的有效质量)
    float m_bias = 0.0f;       // Baumgarte 速度偏置
};