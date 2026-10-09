#pragma once // 记得加上这个，防止重复包含
#include <vector>
#include "Body.h"
#include "../Common/Vector2.h"
#include "../Common/Setting.h"
#include "../Collision/Manifold.h"
#include "../Collision/BroadPhase.h"
#include <map>
#include "../Collision/Contact.h"
#include "ContactManager.h"
#include "Island.h"
#include "../Utils/Profiler.h"
#include "../Events/ContactListener.h"
#include "Joint.h"
/// @brief 物理世界：引擎的总入口
///
/// 职责：
///  - 主循环：step(dt) 依次完成 积分 -> 宽相 -> 接触事件 -> CCD -> 岛屿求解 -> 事件回调
///  - 对象管理：工厂创建（createBody/createBox/createCircle/createCapsule/createJoint）、
///    销毁（destroyBody 延迟销毁，回调中安全；destroyJoint 级联清理）
///  - 查询：rayCast（射线检测）、getBodies（刚体列表）、getIslands（岛屿诊断）
///  - 事件：setContactListener 注册 Enter/Stay/Exit 回调；
///    getContactManager 读取本帧生命周期记录
class World
{
	void dispatchContactEvents();
public:
	World(Vector2 gravity = Settings::GRAVITY) : m_gravity(gravity) {}
	void step(float dt);
	
	void addBody(Body* body) {
		m_bodies.push_back(body);
		int32_t proxyId = m_broadPhase.createProxy(body->getAABB(), body);
		body->setProxyId(proxyId);
	}
	const std::vector<Body*>& getBodies( )const { return m_bodies; }

	// ================= 工厂接口（推荐统一用工厂创建，避免手写 new）=================
	/// 工厂：创建刚体并注册进世界。
	/// @param shape   形状指针（Box/Circle/Capsule/Polygon，材质在 shape->material 中）
	/// @param x, y    初始位置（世界坐标）
	/// @param density 密度：> 0 为动态刚体，= 0 为静态刚体
	/// @return 刚体指针（世界持有所有权）
	Body* createBody(Shape* shape, float x, float y, float density);
	/// 工厂：创建矩形刚体（Box）
	Body* createBox(float w, float h, float x, float y, float density,
		const Physics2D::Material& mat = Physics2D::Material());
	/// 工厂：创建圆形刚体（Circle）
	Body* createCircle(float radius, float x, float y, float density,
		const Physics2D::Material& mat = Physics2D::Material());
	/// 工厂：创建胶囊刚体（Capsule）
	Body* createCapsule(float radius, float length, float x, float y, float density,
		const Physics2D::Material& mat = Physics2D::Material());
	void rayCast(Vector2 p1, Vector2 p2);
	void removeBody(Body* body);
	// 延迟销毁：可在接触回调中安全调用，物体在本帧 step 末尾（或下帧开头）真正移除
	void destroyBody(Body* body);
	void buildAndSolveIslands(float dt);
	inline const std::map<std::pair<Body*, Body*>, Contact*>& getContactMap() const {return m_contactMap;}
	BroadPhase& getBroadPhase() { return m_broadPhase; }
	void wakeNeighbors(Body* body);
	void updateTOI(Contact* c, float dt);
	inline Profiler& getProfiler() { return m_profiler; }
	// 接触生命周期事件管理（Enter/Stay/Exit），每帧 step 后从这里读取记录
	inline ContactManager& getContactManager() { return m_contactManager; }
	inline void setContactListener(ContactListener* listener) { m_contactListener = listener; }
	// 注册关节：把关节同时挂到关联刚体的关节链表上（齿轮关节额外挂父关节的 A 侧）
	void add(Joint* joint);
	// 销毁关节：从世界和双方刚体移除并释放
	void destroyJoint(Joint* joint);
	// 工厂：按 def 类型创建关节并注册到世界（distance 等）
	Joint* createJoint(const JointDef& def);
	inline int getJointCount() const { return (int)m_joints.size(); }
	// 上一帧构建的岛屿（供测试/诊断检查 DFS 连通性）
	inline const std::vector<Island>& getIslands() const { return m_islands; }
private:
	void addContactToGraph(Contact* c);
	void removeContactFromGraph(Contact* c);
	void solveTOI(Contact* contact, float dt);
	void updateAllContactsAndTOI(float dt);
	void updateNeighborsTOI(Body* b, float dt);
	void handleNewCollision(void* uA, void* uB, float dt);
	void updateBroadPhase(float dt);
	// 清理已经完全分离的接触对（AABB 不再重叠）
	void destroySeparatedContacts();
	// 处理延迟销毁队列（把 destroyBody 排队的物体真正移除）
	void flushDestroyQueue();
	std::vector<Body*> m_bodies;
	Vector2 m_gravity;
	std::vector<Manifold> m_manifolds;
	BroadPhase m_broadPhase; // 宽相管理系统
	std::map<std::pair<Body*, Body*>, Contact*> m_contactMap;
	ContactManager m_contactManager; // 接触生命周期事件管理
	std::vector<Body*> m_destroyQueue; // 延迟销毁队列（回调中 destroyBody 安全）
	std::vector<Island> m_islands; // 每帧构建的岛屿集合（供测试检查连通性）
	Profiler m_profiler;
	ContactListener* m_contactListener = nullptr; // 用户自定义的接触事件监听器

	std::vector<Joint*> m_joints;
};