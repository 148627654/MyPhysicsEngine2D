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

class Island
{
public:
    Island(int bodyCapacity, int contactCapacity);
    ~Island() {};
    void clear() {
        m_bodies.clear();
        m_contacts.clear();
        m_joints.clear();
    }

    void add(Body* b) { m_bodies.push_back(b); }
    void add(Contact* c) { m_contacts.push_back(c); }
    void add(Joint* j) { m_joints.push_back(j); }

    // 供测试/诊断：岛屿内的刚体与关节数量
    int getBodyCount() const { return (int)m_bodies.size(); }
    int getJointCount() const { return (int)m_joints.size(); }

    // 核心：把原本在 World::step 里的计算逻辑搬到这里
    void solve(const TimeStep& step, const Vector2& gravity=Settings::GRAVITY);
private:
	std::vector<Body*> m_bodies;
    std::vector<Contact*> m_contacts;
    std::vector<Joint*> m_joints;
};