#pragma once
#include "Manifold.h"
#include "../Dynamics/Body.h"
#include "Collision.h"

struct ContactEdge
{
    Body* other = nullptr;			//链表指向的“对方”物体
    Contact* contact = nullptr;
    ContactEdge* prev = nullptr;
    ContactEdge* next = nullptr;
};

class Contact
{
public:
    Contact(Body* bodyA, Body* bodyB);

    // 获取 Manifold 供 Solver 使用
    Manifold& GetManifold() { return m_manifold; }
    bool IsTouching() const { return m_touching; }

    inline void update(){ m_touching = Collision::Dispatch(&m_manifold, m_bodyA, m_bodyB); }
    // 两个边缘，分别挂在两个 Body 的链表上
    ContactEdge m_nodeA;
    ContactEdge m_nodeB;
    void PreSolve(float dt);
    Body* m_bodyA;
    Body* m_bodyB;

    bool m_islandFlag = false;
private:
	Manifold m_manifold;
	bool m_touching;
};