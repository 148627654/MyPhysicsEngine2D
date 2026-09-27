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
class World
{
	void DispatchContactEvents();
public:
	World(Vector2 gravity = Settings::GRAVITY) : m_gravity(gravity) {}
	void Step(float dt);
	
	void AddBody(Body* body) { 
		m_bodies.push_back(body); 
		int32_t proxyId = m_broadPhase.CreateProxy(body->GetAABB(), body);
		body->setProxyId(proxyId);
	}
	const std::vector<Body*>& GetBodies( )const { return m_bodies; }
	void RayCast(Vector2 p1, Vector2 p2);
	void RemoveBody(Body* body);
	// 延迟销毁：可在接触回调中安全调用，物体在本帧 Step 末尾（或下帧开头）真正移除
	void DestroyBody(Body* body);
	void BuildAndSolveIslands(float dt);
	inline const std::map<std::pair<Body*, Body*>, Contact*>& getContactMap() const {return m_contactMap;}
	BroadPhase& GetBroadPhase() { return m_broadPhase; }
	void WakeNeighbors(Body* body);
	void UpdateTOI(Contact* c, float dt);
	inline Profiler& GetProfiler() { return m_profiler; }
	// 接触生命周期事件管理（Enter/Stay/Exit），每帧 Step 后从这里读取记录
	inline ContactManager& GetContactManager() { return m_contactManager; }
	inline void setContactListener(ContactListener* listener) { m_contactListener = listener; }
private:
	void AddContactToGraph(Contact* c);
	void RemoveContactFromGraph(Contact* c);
	void SolveTOI(Contact* contact, float dt);
	void UpdateAllContactsAndTOI(float dt);
	void UpdateNeighborsTOI(Body* b, float dt);
	void HandleNewCollision(void* uA, void* uB, float dt);
	void UpdateBroadPhase(float dt);
	// 清理已经完全分离的接触对（AABB 不再重叠）
	void DestroySeparatedContacts();
	// 处理延迟销毁队列（把 DestroyBody 排队的物体真正移除）
	void FlushDestroyQueue();
	std::vector<Body*> m_bodies;
	Vector2 m_gravity;
	std::vector<Manifold> m_manifolds;
	BroadPhase m_broadPhase; // 宽相管理系统
	std::map<std::pair<Body*, Body*>, Contact*> m_contactMap;
	ContactManager m_contactManager; // 接触生命周期事件管理
	std::vector<Body*> m_destroyQueue; // 延迟销毁队列（回调中 DestroyBody 安全）
	Profiler m_profiler;
	ContactListener* m_contactListener = nullptr; // 用户自定义的接触事件监听器
};