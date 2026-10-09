#pragma once
#include "Joint.h"
#include "RevoluteJoint.h"
#include "PrismaticJoint.h"

struct GearJointDef : public JointDef
{
	GearJointDef() { type = JointType::Gear; }
    Joint* joint1 = nullptr; // 被绑定的父关节 1（必须为 Revolute 或 Prismatic）
    Joint* joint2 = nullptr; // 被绑定的父关节 2（必须为 Revolute 或 Prismatic）
    float ratio = 1.0f;      // 齿轮传动比（coord1 与 coord2 的耦合系数）
};

/// @brief 齿轮关节：把两个父关节的广义坐标（转角/位移）按比例耦合
///
/// 约束方程 C = coord1 + ratio·coord2 - constant = 0：
///  - coord 为 Revolute 时取相对转角，Prismatic 时取沿轴位移
///  - 双向等式约束（无截断），持有 4 个关联刚体（两父关节各自的 A/B 侧）
///  - 父关节被销毁时本关节必须由 World 级联销毁（持有裸指针）
class GearJoint : public Joint
{
public:
	GearJoint(const GearJointDef* def);

	void initVelocityConstraints(float dt) override;
	void solveVelocityConstraints() override;
	bool solvePositionConstraints() override;

	Vector2 getAnchorA() const override;
	Vector2 getAnchorB() const override;

	// 父关节引用（World 级联销毁检查用）
	Joint* getJoint1() const { return m_joint1; }
	Joint* getJoint2() const { return m_joint2; }
	float getRatio() const { return m_ratio; }
	// 4 个关联刚体（World::add 图论注册用）
	Body* getBodyC() const { return m_bodyC; }
	Body* getBodyD() const { return m_bodyD; }

private:
	// 父关节当前广义坐标（Revolute=相对转角，Prismatic=沿轴位移）
	float getCoordA() const;
	float getCoordB() const;
	// 按当前位姿重算两组雅可比分量（速度/位置求解共用）
	void recomputeJacobians();
	float computeK() const;
	// 施加标量冲量（速度版 / 位置版）
	void applyImpulse(float lambda);
	void applyPositionCorrection(float lambda);

	Joint* m_joint1;
	Joint* m_joint2;
	JointType m_typeA;  // 父关节 1 类型
	JointType m_typeB;  // 父关节 2 类型
	float m_ratio;
	float m_constant;   // 装配时刻 coord1 + ratio·coord2

	Body* m_bodyC;      // 父关节 1 的 bodyA 侧
	Body* m_bodyD;      // 父关节 2 的 bodyA 侧
	// 基类 m_bodyA/m_bodyB 复用为父关节 1/2 的 bodyB 侧（动态端）

	// --- 解算缓存 ---
	float m_impulse = 0.0f;  // 1 维累计传动冲量（双向，无截断）
	// joint1 侧雅可比（A=父1 bodyB、C=父1 bodyA）：Cdot 项 = J_A·v_A - J_C·v_C
	Vector2 m_JvA, m_JvC;
	float m_JwA = 0.0f, m_JwC = 0.0f;
	// joint2 侧雅可比（B=父2 bodyB、D=父2 bodyA）
	Vector2 m_JvB, m_JvD;
	float m_JwB = 0.0f, m_JwD = 0.0f;

	float m_mass = 0.0f;   // 有效质量 1/K（含 ratio² 缩放）
	float m_bias = 0.0f;   // Baumgarte 速度偏置
};
