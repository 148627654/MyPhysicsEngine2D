#pragma once
#include <vector>
#include "../Dynamics/Body.h"
#include "../Collision/Contact.h"
#include "../Common/Setting.h"
struct TimeStep {
    float dt;
    int velocityIterations;
    int positionIterations;
};

class IsLand
{
public:
    IsLand(int bodyCapacity, int contactCapacity);
    ~IsLand() {};
    void Clear() {
        m_bodies.clear();
        m_contacts.clear();
    }

    void Add(Body* b) { m_bodies.push_back(b); }
    void Add(Contact* c) { m_contacts.push_back(c); }

    // 核心：把原本在 World::Step 里的计算逻辑搬到这里
    void Solve(const TimeStep& step, const Vector2& gravity=Settings::GRAVITY);
private:
	std::vector<Body*> m_bodies;
	std::vector<Contact*> m_contacts;
};