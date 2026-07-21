#pragma once // 记得加上这个，防止重复包含
#include "../Common/Vector2.h"
#include "../Collision/Shape.h"
//#include "../Collision/Manifold.h"
class Body
{
public:
	Body(Shape* s, float x=0.0, float y=0.0, float density = 1.0f)
		: shape(s), position(Vector2(x, y)), rotation(0.0f),
		velocity(Vector2(0, 0)), angularVelocity(0.0f),
		force(Vector2(0, 0)), torque(0.0f), gravityScale(1.0f)
	{
		// 根据形状和密度计算质量属性
		if (density > 0.0f && shape != nullptr) {
			MassData data = shape->ComputeMass(density);
			this->mass = data.mass;
			this->invMass = 1.0f / mass;
			this->inertia = data.inertia;
			this->invInertia = 1.0f / inertia;
		}
		else {
			// 静态物体：质量和惯量视为无穷大，倒数为 0
			this->mass = 0.0f;
			this->invMass = 0.0f;
			this->inertia = 0.0f;
			this->invInertia = 0.0f;
		}
		this->restitution = 0.5f; // 默认弹力 0.5
		this->friction = 0.2f;    // 默认摩擦 0.2
		updateAABB( );
	}
	inline void ClearForce() { force.Clear(); }
	Vector2 AddForce(Vector2 f);
	friend class World;
	// 方便外部（如日志系统）读取数据
	void SetPosition(float x , float y) { position = Vector2(x , y); }
	void SetPosition(const Vector2 &v) { position = v; }
	Vector2 GetPosition() const { return position; }
	Vector2 GetVelocity() const { return velocity; }
	Shape* GetShape()const { return shape; };
	float GetRotation()const { return rotation; }
	void SetRotation(float r) { rotation = r; }
	void SetVelocity(Vector2 v) { velocity = v; }
	AABB GetAABB( )const { return worldAABB; }
	float getAngularVelocity( )const { return angularVelocity; }
	void setAngularVelocity(float av) { angularVelocity = av; }
	inline float getInvInertia( )const { return invInertia; }
	//绑定形状并自动计算质量属性
	void SetShape(Shape* s, float density);
	inline float getRestitution( )const { return restitution; }
	inline void setRestitution(float e) { restitution = e; }
	inline float getInvMass( )const { return invMass; }
	//转矩累加
	float addTorque(float t) { return torque += t; }
	//力作用于非重心位置会产生转矩，公式为：$Torque = (point - position) \times force$ (2D 叉积)。
	void ApplyForceAtPoint(Vector2 force, Vector2 worldPoint);
	void updateAABB( );
	void ApplyImpulse(Vector2 impulse);
	void ApplyImpulse(const Vector2& impulse , const Vector2& contactVector);
	inline float getFriction( ) const { return friction; }
	inline void setFriction(float f) { friction = f; }
	inline int32_t getProxyId() const { return m_proxyId; }
	inline void setProxyId(int32_t id) { m_proxyId = id; }
private:
	Vector2 position;		//当前位置
	Vector2 velocity;		//当前速度
	Vector2 force;			//累积力（每一帧更新前清零）

	float mass;				//质量
	float invMass;			//质量的倒数
	float gravityScale;		//重力的缩放系数

	//角度属性
	float rotation;			//当前角度（弧度）
	float angularVelocity;	//角速度
	float torque;			//累计转矩
	//惯量属性
	float inertia;			//转动惯量。
	float invInertia;		//转动惯量的倒数（如果是静态物体，则为 0）。
	Shape* shape;			//实体

	AABB worldAABB;

	//冲量 牛顿恢复定律
	float restitution;

	float friction;    // 摩擦系数，建议范围 0.0 ~ 1.0 \mu
	int32_t m_proxyId = -1; // 默认 -1 代表还没进树
};