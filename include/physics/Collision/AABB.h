#pragma once // 记得加上这个，防止重复包含
#include "../Common/Vector2.h"
struct AABB {
	Vector2 min; // 左下角
	Vector2 max; // 右上角

	static AABB Union(const AABB& a, const AABB& b);

	// 动态树（BVH）在选择分支时通常使用“周长”作为代价函数，这在 2D 中比面积更稳定
	float GetPerimeter() const;

    // 3. 判断两个 AABB 是否重叠 (宽相检测的基础)
    bool Overlap(const AABB& other) const;

    // 4. 判断一个 AABB 是否完全包含另一个 (后续 Fat AABB 会用到)
	bool Contains(const AABB& other) const;
};