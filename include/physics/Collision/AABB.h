#pragma once // 记得加上这个，防止重复包含
#include "../Common/Vector2.h"
struct AABB {
	Vector2 min; // 左下角
	Vector2 max; // 右上角
};