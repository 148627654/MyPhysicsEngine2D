#pragma once // 记得加上这个，防止重复包含
#include <vector>
#include "Body.h"
#include "../Common/Vector2.h"
#include "../Common/Setting.h"
#include "../Collision/Manifold.h"
#include "../Collision/BroadPhase.h"
class World
{
public:
	World(Vector2 gravity = Settings::GRAVITY) : m_gravity(gravity) {}
	void Step(float dt);
	void AddBody(Body* body) { 
		m_bodies.push_back(body); 
		int32_t proxyId = m_broadPhase.CreateProxy(body->GetAABB(), body);
		body->setProxyId(proxyId);
	}
	const std::vector<Body*>& GetBodies( )const { return m_bodies; }
private:
	std::vector<Body*> m_bodies;
	Vector2 m_gravity;
	std::vector<Manifold> m_manifolds;
	BroadPhase m_broadPhase; // 宽相管理系统
};