#pragma once // 记得加上这个，防止重复包含
#include <vector>
#include "Body.h"
#include "../Common/Vector2.h"
#include "../Common/Setting.h"
class World
{
public:
	World(Vector2 gravity = Settings::GRAVITY) : m_gravity(gravity) {}
	void Step(float dt);
	void AddBody(Body* body) { m_bodies.push_back(body); }

private:
	std::vector<Body*> m_bodies;
	Vector2 m_gravity;
};