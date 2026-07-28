#pragma once // 记得加上这个，防止重复包含
#include "../Common/Vector2.h"
#include "../Collision/Shape.h"
//#include "../Collision/Manifold.h"
class Contact;
struct ContactEdge;


class Body
{
public:
	Body(Shape* s, float x = 0.0, float y = 0.0, float density = 1.0f)
		: shape(s), position(Vector2(x, y)), rotation(0.0f),
		velocity(Vector2(0, 0)), angularVelocity(0.0f),
		force(Vector2(0, 0)), torque(0.0f), gravityScale(1.0f)
	{
		if (density > 0.0f && shape != nullptr) {
			// --- 动态物体逻辑 ---
			MassData data = shape->ComputeMass(density);
			this->mass = data.mass;
			this->invMass = 1.0f / mass;
			this->inertia = data.inertia;
			this->invInertia = 1.0f / inertia;

			m_isAwake = true;           // 动态物体初始是醒着的
			m_isSleepAllowed = true;    // 允许休眠
		}
		else {
			// --- 静态物体逻辑 ---
			this->mass = 0.0f;
			this->invMass = 0.0f;
			this->inertia = 0.0f;
			this->invInertia = 0.0f;

			m_isAwake = false;          // 静态物体永远“不醒”（不主动触发解算）
			m_isSleepAllowed = false;   // 静态物体不需要休眠逻辑
		}

		this->restitution = 0.5f;
		this->friction = 0.2f;
		this->m_sleepTimer = 0.0f;      // 计时器清零
		updateAABB();
	}
	inline void ClearForce() { force.Clear(); }
	Vector2 AddForce(Vector2 f);
	
	// 方便外部（如日志系统）读取数据
	void SetPosition(float x, float y);
	void SetPosition(const Vector2& v);
	Vector2 GetPosition() const { return position; }
	Vector2 GetVelocity() const { return velocity; }
	Shape* GetShape()const { return shape; };
	float GetRotation()const { return rotation; }
	void SetRotation(float r);
	void SetVelocity(Vector2 v) { velocity = v;}
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
	ContactEdge* getContactList() { return m_contactList; }
	inline Vector2 getForce() const { return force; }
	inline void setForce(Vector2 f) { force = f; }
	inline float getGravityScale() const { return gravityScale; }
	inline void setGravityScale(float g) { gravityScale = g; }
	inline float getTorque() { return torque; }
	inline void setTorque(float t) { torque = t; }
	void setAwake(bool w);
	inline bool IsAwake() const { return m_isAwake; }
	inline bool IsSleepAllow() const { return m_isSleepAllowed; }
	inline void setSleepAllow(bool a) { m_isSleepAllowed = a; }
	inline float getSleepTimer() const { return m_sleepTimer; }
	inline void setSleepTimer(float time) { m_sleepTimer = time; }
	
private:
	friend struct ContactEdge;
	friend class World;

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

	ContactEdge* m_contactList = nullptr;
	//岛屿的遍历
	bool m_islandFlag = false;
	bool m_isAwake;			//是否处于清醒状态。
	bool m_isSleepAllowed;	//是否允许该物体休眠。
	float m_sleepTimer;				//当前物体处于低能量状态的持续时间。
};