#pragma once // 记得加上这个，防止重复包含
#include "../Common/Vector2.h"
#include "../Collision/Shape.h"
#include "../Common/Setting.h"
#include <vector>
//#include "../Collision/Manifold.h"
class Contact;
struct ContactEdge;
enum class BodyType {
	Static,
	Dynamic
};

struct Transform {
	Vector2 p;      // 位置 (Position)
	float q;        // 旋转弧度 (Angle/Rotation) - 也可以是旋转矩阵，2D 简单起见用 float

	Transform() : p(0, 0), q(0) {}
	Transform(Vector2 pos, float angle) : p(pos), q(angle) {}
};
class Joint;

/// @brief 刚体：引擎中的动力学实体
///
/// 职责：
///  - 运动状态：位置/角度/线速度/角速度（position、rotation、velocity、angularVelocity）
///  - 质量属性：质量/惯量及其倒数（静态刚体倒数为 0）
///  - 力与冲量：addForce / applyImpulse / applyForceAtPoint / addTorque
///  - 睡眠：低速自动入睡（setAwake / setSleepAllowed / forceSleep）
///  - 材质透传：getRestitution / getFriction 等直接读写 shape->material（唯一数据源）
///
/// 创建方式：优先使用 World 工厂（createBody / createBox / createCircle / createCapsule）
class Body
{
public:
	Body(Shape* s, float x, float y, float density)
		: shape(s),
		position(Vector2(x, y)),
		rotation(0.0f),
		velocity(Vector2(0, 0)),
		angularVelocity(0.0f),
		force(Vector2(0, 0)),
		torque(0.0f),
		gravityScale(1.0f),
		m_proxyId(-1),
		m_contactList(nullptr),
		m_islandFlag(false),
		m_sleepTimer(0.0f),
		m_isBullet(false),
		// --- CCD 关键初始化 ---
		m_prevPosition(Vector2(x, y)), // 初始位置与上一帧位置同步
		m_prevRotation(0.0f)           // 初始角度与上一帧角度同步
	{
		// density 以 Body 构造参数为准，写回材质作为唯一数据源
		if (shape != nullptr) shape->material.density = density;

		if (density > 0.0f && shape != nullptr) {
			MassData data = shape->computeMass(density);
			this->mass = data.mass;
			this->invMass = 1.0f / mass;
			this->inertia = data.inertia;
			this->invInertia = 1.0f / inertia;
			m_isAwake = true;
			m_isSleepAllowed = true;
		}
		else {
			this->mass = 0.0f;
			this->invMass = 0.0f;
			this->inertia = 0.0f;
			this->invInertia = 0.0f;
			m_isAwake = false;
			m_isSleepAllowed = false;
		}

		// 初始化 AABB
		updateAABB();
	}
	inline void clearForce() { force.clear(); }
	Vector2 addForce(Vector2 f);
	
	// 方便外部（如日志系统）读取数据
	void setPosition(float x, float y);
	void setPosition(const Vector2& v);
	// 位置修正专用：静默修改位置（不唤醒、不重置睡眠计时），供 Solver 的 positionalCorrection 使用
	void setPositionQuiet(const Vector2& v);
	// 关节位置修正专用：静默修改角度
	void setRotationQuiet(float r);
	Vector2 getPosition() const { return position; }
	Vector2 getVelocity() const { return velocity; }
	Shape* getShape()const { return shape; };
	float getRotation()const { return rotation; }
	void setRotation(float r);
	void setVelocity(Vector2 v) { velocity = v;}
	AABB getAABB( )const { return worldAABB; }
	float getAngularVelocity( )const { return angularVelocity; }
	void setAngularVelocity(float av) { angularVelocity = av; }
	inline float getInvInertia( )const { return invInertia; }
	//绑定形状并自动计算质量属性
	void setShape(Shape* s, float density);
	// 重新根据 shape->material.density 计算质量属性（运行时改材质密度后调用）
	void updateMassData();
	// --- 材质属性：存储在 Shape::material 中，这里仅为兼容旧调用点做透传 ---
	inline float getRestitution( )const { return shape ? shape->material.restitution : 0.0f; }
	inline void setRestitution(float e) { if (shape) shape->material.restitution = e; }
	inline float getInvMass( )const { return invMass; }
	inline float getMass( )const { return mass; }
	inline float getInertia( )const { return inertia; }
	//转矩累加
	float addTorque(float t) { return torque += t; }
	//力作用于非重心位置会产生转矩，公式为：$Torque = (point - position) \times force$ (2D 叉积)。
	void applyForceAtPoint(Vector2 force, Vector2 worldPoint);
	void updateAABB( );
	void applyImpulse(Vector2 impulse);
	void applyImpulse(const Vector2& impulse , const Vector2& contactVector);
	inline float getFriction( ) const { return shape ? shape->material.dynamicFriction : 0.0f; }
	inline void setFriction(float f) { if (shape) shape->material.dynamicFriction = f; }
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
	inline bool isAwake() const { return m_isAwake; }
	inline bool isSleepAllowed() const { return m_isSleepAllowed; }
	inline void setSleepAllow(bool a) { m_isSleepAllowed = a; }
	inline float getSleepTimer() const { return m_sleepTimer; }
	inline void setSleepTimer(float time) { m_sleepTimer = time; }
	void setType(BodyType type, float density = 1.0f);
	void forceSleep();
	AABB getSweptAABB(float dt)const;
	void setBullet(bool flag) { m_isBullet = flag; }
	bool isBullet() const { return m_isBullet; }
	Transform getTransform(float alpha, float dt=Settings::DT);
	void setTransform(const Vector2& position, float angle);
	void setTransform(const Transform& tf);
	void savePrevState();

	// 供 World/Joint 在绑定时调用
	inline void addJoint(Joint* joint) {m_joints.push_back(joint);}

	void removeJoint(Joint* joint);

	inline const std::vector<Joint*>& getJointList() const {return m_joints;}

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

	int32_t m_proxyId = -1; // 默认 -1 代表还没进树

	ContactEdge* m_contactList = nullptr;
	//岛屿的遍历
	bool m_islandFlag = false;
	bool m_isAwake;			//是否处于清醒状态。
	bool m_isSleepAllowed;	//是否允许该物体休眠。
	float m_sleepTimer;				//当前物体处于低能量状态的持续时间。

	bool m_isBullet;		// 是否高速物体，需要连续碰撞检测

	Vector2 m_prevPosition;
	float m_prevRotation;

	std::vector<Joint*> m_joints; // 存储与当前 Body 相连的所有关节
};