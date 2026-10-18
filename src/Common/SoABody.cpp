#include "SoABody.h"
#include "../Dynamics/Body.h"

void BodySoA::MoveSlot(int from, int to)
{
    positions[to] = positions[from];
    velocities[to] = velocities[from];
    rotations[to] = rotations[from];
    angularVels[to] = angularVels[from];
    invMasses[to] = invMasses[from];
    invInertias[to] = invInertias[from];
    forces[to] = forces[from];
    torques[to] = torques[from];
    gravityScales[to] = gravityScales[from];
    flags[to] = flags[from];

    // 极其关键：通知那个"被搬家的倒霉蛋刚体"槽位句柄已变更
    Body* moved = bodyOwners[from];
    bodyOwners[to] = moved;
    moved->m_soaIndex = to;
}

int BodySoA::AllocateSlot(Body* body)
{
    int index = (int)bodyOwners.size();

    // 1. 追加全部平行数组，写入刚体热数据快照（Body 是友元，可直读私有成员）
    positions.push_back(body->position);
    velocities.push_back(body->velocity);
    rotations.push_back(body->rotation);
    angularVels.push_back(body->angularVelocity);
    invMasses.push_back(body->invMass);
    invInertias.push_back(body->invInertia);
    forces.push_back(body->force);
    torques.push_back(body->torque);
    gravityScales.push_back(body->gravityScale);
    flags.push_back((body->m_isAwake ? FLAG_AWAKE : 0u) |
                    (body->m_isBullet ? FLAG_BULLET : 0u));
    bodyOwners.push_back(body);

    // 2. 反向写回槽位句柄：此后 Body 的所有热数据访问器无缝穿透到本槽位
    body->m_soa = this;
    body->m_soaIndex = index;
    return index;
}

void BodySoA::FreeSlot(int index)
{
    if (index < 0 || index >= (int)bodyOwners.size())
        return;

    int last = (int)bodyOwners.size() - 1;
    Body* removed = bodyOwners[index];

    // 1. Swap-and-Pop：末尾槽位整体搬运覆盖被删槽位（O(1)，无空洞）
    if (index != last) {
        MoveSlot(last, index);
    }

    // 2. 截断全部平行数组尾部
    positions.pop_back();
    velocities.pop_back();
    rotations.pop_back();
    angularVels.pop_back();
    invMasses.pop_back();
    invInertias.pop_back();
    forces.pop_back();
    torques.pop_back();
    gravityScales.pop_back();
    flags.pop_back();
    bodyOwners.pop_back();

    // 3. 被移除刚体彻底脱离 SoA（防止 delete 后残留句柄指向复用槽位）
    removed->m_soa = nullptr;
    removed->m_soaIndex = -1;
}
