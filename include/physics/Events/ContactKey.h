#pragma once

#include "../Dynamics/Body.h"
#include "../Collision/Manifold.h"
#include <cstdint>
#include <cstddef>
#include <algorithm>
enum class ContactState
{
	Enter,			// 刚发生碰撞
	Stay,			// 持续碰撞
	Exit,			// 刚结束碰撞
};

struct ContactKey
{
	int32_t idA;			// 碰撞体 A 的 ID
	int32_t idB;			// 碰撞体 B 的 ID

	ContactKey(uint32_t a, uint32_t b) {
		idA = std::min(a, b);
		idB = std::max(a, b);
	}

	bool operator==(const ContactKey& o) const {
		return idA == o.idA && idB == o.idB;
	}
};

struct ContactKeyHash {
	std::size_t operator()(const ContactKey& k) const {
		// 将两个 32 位 ID 压缩为一个 64 位哈希值
		return (static_cast<uint64_t>(k.idA) << 32) | static_cast<uint64_t>(k.idB);
	}
};

struct ContactRecord
{
	Body* bodyA=nullptr;
	Body* bodyB=nullptr;
	bool isTrigger=false;
	ContactState state=ContactState::Enter;
	Manifold manifold;

	ContactRecord() = default;
};