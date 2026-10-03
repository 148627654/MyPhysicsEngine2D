// GoogleTest 版 V3/007：关节系统测试
// 移植自 tests/V3/007.cpp，场景逻辑原样保留，断言改为 gtest 宏
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "Box.h"
#include "Logger.h"

// MockJoint：统计解算管线各阶段的调用次数
class MockJoint : public Joint {
public:
    int initCalls = 0;     // initVelocityConstraints 调用次数
    int solveCalls = 0;    // solveVelocityConstraints 调用次数
    int positionCalls = 0; // solvePositionConstraints 调用次数

    MockJoint(const JointDef& def) : Joint(&def) {}

    void initVelocityConstraints(float dt) override { initCalls++; }
    void solveVelocityConstraints() override { solveCalls++; }
    bool solvePositionConstraints() override { positionCalls++; return true; }
    Vector2 getAnchorA() const override { return Vector2(0, 0); }
    Vector2 getAnchorB() const override { return Vector2(0, 0); }
};

// 动态 Box 刚体（零重力，防止自由落体干扰关节测试）
static Body* MakeBoxBody(float x, float y) {
    Body* b = new Body(new Box(1.0f, 1.0f), x, y, 1.0f);
    b->setGravityScale(0.0f);
    return b;
}

// --- 场景 1: 关节图连通性测试 (Island Connectivity) ---
// 相距很远的 A、B（AABB 完全不相交）仅靠关节连结，DFS 必须把它们归入同一岛屿
TEST(V3_007, IslandConnectivity) {
    World world(Vector2(0, -9.8f));

    Body* A = MakeBoxBody(0.0f, 0.0f);
    Body* B = MakeBoxBody(100.0f, 0.0f); // 相距 100 米，绝无碰撞
    world.addBody(A);
    world.addBody(B);

    JointDef def;
    def.type = JointType::Distance;
    def.bodyA = A;
    def.bodyB = B;
    world.add(new MockJoint(def));

    world.step(1.0f / 60.0f);

    // 断言：DFS 把 A 和 B 归入同一个 Island（1 岛屿、2 刚体、1 关节）
    const auto& islands = world.getIslands();
    ASSERT_EQ(islands.size(), 1u);
    EXPECT_EQ(islands[0].getBodyCount(), 2);
    EXPECT_EQ(islands[0].getJointCount(), 1);
}

// --- 场景 2: 约束解算管线被调测试 (Solver Pipeline dispatch) ---
// 1 帧模拟（速度迭代 8 次）：Init 恰好 1 次，solve 严格连续 8 次
TEST(V3_007, SolverPipelineDispatch) {
    World world(Vector2(0, -9.8f));

    Body* A = MakeBoxBody(0.0f, 0.0f);
    Body* B = MakeBoxBody(5.0f, 0.0f);
    world.addBody(A);
    world.addBody(B);

    JointDef def;
    def.type = JointType::Distance;
    def.bodyA = A;
    def.bodyB = B;
    MockJoint* joint = new MockJoint(def);
    world.add(joint);

    world.step(1.0f / 60.0f);

    EXPECT_EQ(joint->initCalls, 1);
    EXPECT_EQ(joint->solveCalls, 10); // 速度迭代已提升至 10（Day 13 规格）
}

// --- 场景 3: 刚体自毁级联测试 (Cascade Destruction) ---
// destroyBody(A)：关节自动归零、B 的关节列表清空、后续模拟无野指针崩溃
TEST(V3_007, CascadeDestruction) {
    World world(Vector2(0, -9.8f));

    Body* A = MakeBoxBody(0.0f, 0.0f);
    Body* B = MakeBoxBody(5.0f, 0.0f);
    world.addBody(A);
    world.addBody(B);

    JointDef def;
    def.type = JointType::Distance;
    def.bodyA = A;
    def.bodyB = B;
    world.add(new MockJoint(def));

    world.destroyBody(A);

    // 断言：关节数立刻变为 0，B 的 JointEdge 列表被清空
    EXPECT_EQ(world.getJointCount(), 0);
    EXPECT_TRUE(B->getJointList().empty());

    // 后续模拟多帧，绝不产生野指针崩溃（能走完循环即无崩溃）
    for (int i = 0; i < 30; ++i) world.step(1.0f / 60.0f);
    EXPECT_EQ(world.getBodies().size(), 1u); // 世界只剩 B
}
