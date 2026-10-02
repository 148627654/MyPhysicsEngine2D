#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Dynamics/Joint.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"

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
bool RunIslandConnectivityTest() {
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
    bool ok = islands.size() == 1
        && islands[0].getBodyCount() == 2
        && islands[0].getJointCount() == 1;

    Logger::info("IslandConnectivity: islands=" + std::to_string(islands.size()) +
        " bodies=" + std::to_string(islands.empty() ? 0 : islands[0].getBodyCount()) +
        " joints=" + std::to_string(islands.empty() ? 0 : islands[0].getJointCount()) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 2: 约束解算管线被调测试 (Solver Pipeline dispatch) ---
// 1 帧模拟（速度迭代 8 次）：Init 恰好 1 次，solve 严格连续 8 次
bool RunSolverPipelineTest() {
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

    bool ok = (joint->initCalls == 1) && (joint->solveCalls == 8);

    Logger::info("SolverPipeline: init=" + std::to_string(joint->initCalls) +
        " solve=" + std::to_string(joint->solveCalls) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: 刚体自毁级联测试 (Cascade Destruction) ---
// destroyBody(A)：关节自动归零、B 的关节列表清空、后续模拟无野指针崩溃
bool RunCascadeDestructionTest() {
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
    bool ok = (world.getJointCount() == 0) && B->getJointList().empty();

    // 后续模拟多帧，绝不产生野指针崩溃（能走完循环即无崩溃）
    for (int i = 0; i < 30; ++i) world.step(1.0f / 60.0f);
    ok &= (world.getBodies().size() == 1); // 世界只剩 B

    Logger::info("CascadeDestruction: joints=" + std::to_string(world.getJointCount()) +
        " B_jointList=" + std::to_string(B->getJointList().size()) +
        " bodies=" + std::to_string(world.getBodies().size()) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

//int main() {
//    Logger::info(">>> Starting V3 007: Joint Test...");
//    bool ok = true;
//    ok &= RunIslandConnectivityTest();
//    ok &= RunSolverPipelineTest();
//    ok &= RunCascadeDestructionTest();
//    Logger::info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
//    return ok ? 0 : 1;
//}
