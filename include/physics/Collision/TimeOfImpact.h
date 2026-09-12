#pragma once
#include "../Dynamics/Body.h"
#include "Box.h"
#include "Circle.h"
// 描述 TOI 计算所需的全部外部条件
struct TOIInput {
    Body* bodyA=nullptr;
    Body* bodyB=nullptr;
    float dt=Settings::DT;           // 这一帧的时间步长
    float tolerance;    // 容差，比如 0.001，当距离小于它时认为撞击
    int maxIterations=20;  // 最大迭代次数，防止死循环
};

// 描述计算结果
struct TOIOutput {
    enum State { Unknown, Hit, Separated, Overlapped };
    State state;
    float alpha;
    Vector2 normal; // <--- 新增：存储撞击那一刻的法线
};

class TimeOfImpact {
public:
    // TimeOfImpact::Solve(input)
    static TOIOutput Solve(const TOIInput& input);

private:
    // 计算两物体在特定 Transform 下的最近距离
    static float CalculateDistance(const TOIInput& input, float alpha, Vector2& outNormal);

    static float CircleVsCircleDistance(Circle* a, const Transform& xfA, Circle* b, const Transform& xfB, Vector2& outNormal);
    static float BoxVsCircleDistance(Box* box, const Transform& xfBox, Circle* circle, const Transform& xfCircle, Vector2& outNormal);
    static float BoxVsBoxDistance(Box* a, const Transform& xfA, Box* b, const Transform& xfB, Vector2& outNormal);
};