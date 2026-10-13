#pragma once
#include "../Common/Vector2.h"

class Body;
class Shape;

/// @brief 射线检测观察者回调（World::RayCast 多重回调接口）
///
/// 返回值控制遍历行为（动态收缩剪枝的核心）：
///  - 返回 0：立即终止全部检测；
///  - 返回当前 fraction：收缩射线继续寻找更近的阻挡（最近命中优先，树层剪枝）；
///  - 返回 1：继续穿透，收集射线上全部命中。
class RayCastCallback {
public:
    virtual ~RayCastCallback() = default;
    virtual float ReportHit(Body* body, Shape* shape, const Vector2& point,
        const Vector2& normal, float fraction) = 0;
};

/// @brief 区域重叠查询观察者回调（World::QueryAABB 回调接口）
///
/// 返回 false 提前终止区域搜索（如 AOE 伤害统计到上限即停）。
class QueryCallback {
public:
    virtual ~QueryCallback() = default;
    virtual bool ReportBody(Body* body) = 0;
};
