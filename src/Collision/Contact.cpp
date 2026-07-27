#include "Contact.h"

Contact::Contact(Body* bodyA, Body* bodyB)
    : m_bodyA(bodyA), m_bodyB(bodyB), m_manifold(bodyA, bodyB), m_touching(false)
{
    m_nodeA.contact = this;
    m_nodeA.other = bodyB;
    m_nodeA.prev = nullptr;
    m_nodeA.next = nullptr;

    m_nodeB.contact = this;
    m_nodeB.other = bodyA;
    m_nodeB.prev = nullptr;
    m_nodeB.next = nullptr;

    m_islandFlag = false;
}
