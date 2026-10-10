#pragma once
#include "Joint.h"
#include "../Common/Vector2.h"

// 车轮关节定义：刚性硬悬挂 + 轮毂驱动马达的一体化载具关节。
// 相比 V3 的 Spring+Revolute 拼装方案：单关节一个解算槽位、轮轴几何强锁死（绝对不脱轴）
struct WheelJointDef : public JointDef
{
	WheelJointDef() { type = JointType::Wheel; }
    Vector2 localAnchorA;                       // 底盘悬挂点（BodyA 局部坐标）
    Vector2 localAnchorB;                       // 轮心（BodyB 局部坐标）
    Vector2 localAxisA = Vector2(0.0f, -1.0f);  // 底盘坐标系下的悬挂轴向（默认向下 (0,-1)）
    bool enableMotor = false;                   // 是否开启轮毂马达
    float motorSpeed = 0.0f;                    // 目标驱动转速 (rad/s)
    float maxMotorTorque = 0.0f;                // 马达最大输出扭矩 (N·m)

    // 装配助手：世界锚点（悬挂点）+ 底盘局部悬挂轴 → 局部锚点/局部轴
    void Initialize(Body* chassis, Body* wheel, const Vector2& anchor, const Vector2& axis) {
        bodyA = chassis;
        bodyB = wheel;
        localAnchorA = (anchor - chassis->getPosition()).rotate(-chassis->getRotation());
        localAnchorB = (anchor - wheel->getPosition()).rotate(-wheel->getRotation());
        localAxisA = axis; // 传入的是底盘局部系下的悬挂轴向
    }
};

class WheelJoint : public Joint
{
public:
	WheelJoint(const WheelJointDef* def);
	void initVelocityConstraints(float dt) override;
	void solveVelocityConstraints() override;
	bool solvePositionConstraints() override;
	Vector2 getAnchorA() const override;
	Vector2 getAnchorB() const override;

	// 底盘局部悬挂轴向（诊断/复装用）
	Vector2 getLocalAxisA() const { return m_localXAxisA; }

	// --- 运行时马达控制 API（载具游戏常用）---
	void setMotorSpeed(float speed) { m_motorSpeed = speed; }
	void setMaxMotorTorque(float torque) { m_maxMotorTorque = torque; }
	void enableMotor(bool flag) { m_enableMotor = flag; if (!flag) m_motorImpulse = 0.0f; }
	// 本帧马达输出的等效扭矩 (N·m)
	float getMotorTorque(float dt) const {
		return (dt > 0.0f) ? m_motorImpulse / dt : 0.0f;
	}
private:
    Vector2 m_localAnchorA;
    Vector2 m_localAnchorB;
    Vector2 m_localXAxisA;   // 悬挂轴向 s（底盘局部，已归一化）
    Vector2 m_localYAxisA;   // 垂向正交轴 n = s^⊥（底盘局部）

    // --- 2-DOF 刚性轮轴线约束 ---
    Vector2 m_linearImpulse = { 0.0f, 0.0f };  // 2D 累计线冲量 [λn, λs]（热启动）
    Vector2 m_axis, m_perp;   // 世界悬挂轴 s 与正交轴 n（随底盘旋转）
    Vector2 m_rA, m_rB;       // 世界力矩臂
    float m_sA, m_sB;         // r × s 力矩臂投影
    float m_nA, m_nB;         // r × n 力矩臂投影
    // 2×2 有效质量矩阵 K（对称）及其克莱姆法则逆
    float m_K11 = 0.0f, m_K12 = 0.0f, m_K22 = 0.0f;
    float m_invK11 = 0.0f, m_invK12 = 0.0f, m_invK22 = 0.0f;
    Vector2 m_bias = { 0.0f, 0.0f };           // 稳定化速度偏置（引擎约定：恒 0，见 cpp 注释）

    // --- 1-DOF 轮毂驱动马达（与线冲量独立记账）---
    bool m_enableMotor = false;
    float m_motorSpeed = 0.0f;
    float m_maxMotorTorque = 0.0f;
    float m_motorImpulse = 0.0f;     // 独立缓存马达扭矩冲量
    float m_maxMotorImpulse = 0.0f;  // 每帧钳位上限 = dt · τmax
    float m_angularMass = 0.0f;      // 角有效质量 = 1 / (iA + iB)
};
