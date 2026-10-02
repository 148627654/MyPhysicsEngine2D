// GoogleTest 版 V3/008：DistanceJoint 刚性杆测试
// 移植自 tests/V3/008.cpp，场景逻辑原样保留，断言改为 gtest 宏
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "DistanceJoint.h"
#include "Box.h"
#include "Logger.h"
#include <cmath>

// --- 场景 1: 刚性抗拉压测试 (Push & Pull Rigidity) ---
// 静态天花板 A(0,10) 挂 1kg 重物 B(0,5)，杆长 5.0，重力下 300 帧后误差必须 < 1e-3
TEST(V3_008, PushPullRigidity) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* ceiling = new Body(new Box(2.0f, 1.0f), 0.0f, 10.0f, 0.0f); // 静态天花板
    world.addBody(ceiling);
    Body* weight = new Body(new Box(1.0f, 1.0f), 0.0f, 5.0f, 1.0f);   // 1kg 重物
    weight->setSleepAllow(false); // 禁止入睡，让断言完全由关节约束保证
    world.addBody(weight);

    DistanceJointDef def;
    def.bodyA = ceiling;
    def.bodyB = weight;
    def.localAnchorA = Vector2(0.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.length = 5.0f;
    world.createJoint(def);

    for (int i = 0; i < 300; ++i) world.step(dt);

    float dist = (weight->getPosition() - ceiling->getPosition()).length();
    float error = std::abs(dist - 5.0f);
    EXPECT_LT(error, 1e-3f); // 极高刚性：无视觉拉伸
}

// --- 场景 2: 单摆周期与机械能守恒测试 ---
// B 从水平位置 (5,10) 释放，绕静态点 (0,10) 摆动；任意时刻到支点距离恒等于 5.0
TEST(V3_008, PendulumPerfectArc) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* pivot = new Body(new Box(1.0f, 1.0f), 0.0f, 10.0f, 0.0f); // 静态支点
    world.addBody(pivot);
    Body* bob = new Body(new Box(1.0f, 1.0f), 5.0f, 10.0f, 1.0f);   // 摆锤（水平释放）
    world.addBody(bob);

    DistanceJointDef def;
    def.bodyA = pivot;
    def.bodyB = bob;
    def.localAnchorA = Vector2(0.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.0f);
    def.length = 5.0f;
    world.createJoint(def);

    float maxDeviation = 0.0f;
    for (int i = 0; i < 100; ++i) {
        world.step(dt);
        float dist = (bob->getPosition() - pivot->getPosition()).length();
        maxDeviation = std::max(maxDeviation, std::abs(dist - 5.0f));
    }

    // 轨迹是完美圆弧：与固定点距离恒等于 5.0
    EXPECT_LT(maxDeviation, 0.05f);
}

// --- 场景 3: 热启动 (Warm Starting) 收敛性对比 ---
// 静态悬挂状态下对比首轮迭代残差：开启热启动后误差必须下降 90% 以上
TEST(V3_008, WarmStartResidualReduction) {
    float dt = 1.0f / 60.0f;

    // 场景 A：开启热启动
    World worldWarm(Vector2(0, -9.8f));
    Body* ceilingW = new Body(new Box(2.0f, 1.0f), 0.0f, 10.0f, 0.0f);
    worldWarm.addBody(ceilingW);
    Body* weightW = new Body(new Box(1.0f, 1.0f), 0.0f, 5.0f, 1.0f);
    weightW->setSleepAllow(false);
    worldWarm.addBody(weightW);
    DistanceJointDef defWarm;
    defWarm.bodyA = ceilingW; defWarm.bodyB = weightW;
    defWarm.length = 5.0f;
    defWarm.enableWarmStart = true;
    DistanceJoint* jointWarm = static_cast<DistanceJoint*>(worldWarm.createJoint(defWarm));

    // 场景 B：关闭热启动
    World worldCold(Vector2(0, -9.8f));
    Body* ceilingC = new Body(new Box(2.0f, 1.0f), 0.0f, 10.0f, 0.0f);
    worldCold.addBody(ceilingC);
    Body* weightC = new Body(new Box(1.0f, 1.0f), 0.0f, 5.0f, 1.0f);
    weightC->setSleepAllow(false);
    worldCold.addBody(weightC);
    DistanceJointDef defCold;
    defCold.bodyA = ceilingC; defCold.bodyB = weightC;
    defCold.length = 5.0f;
    defCold.enableWarmStart = false;
    DistanceJoint* jointCold = static_cast<DistanceJoint*>(worldCold.createJoint(defCold));

    // 各自稳定 120 帧后取"首轮迭代前残差"
    for (int i = 0; i < 120; ++i) {
        worldWarm.step(dt);
        worldCold.step(dt);
    }

    float residualWarm = std::abs(jointWarm->getFirstResidual());
    float residualCold = std::abs(jointCold->getFirstResidual());

    // 开启热启动后，首轮迭代误差下降 90% 以上
    EXPECT_GT(residualCold, 1e-4f);
    EXPECT_LT(residualWarm, residualCold * 0.1f);
}
