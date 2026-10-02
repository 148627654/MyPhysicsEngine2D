#include "../Dynamics/World.h"
#include "../Dynamics/Body.h"
#include "../Dynamics/Joint.h"
#include "../Dynamics/DistanceJoint.h"
#include "../Collision/Box.h"
#include "../Utils/Logger.h"
#include <cmath>

// --- 场景 1: 简谐振动机械能守恒测试 (Undamped Harmonic Oscillation) ---
// frequencyHz=2.0、dampingRatio=0.0，拉长 2 米释放，自由振荡 120 帧
bool RunHarmonicTest() {
    World world(Vector2(0, 0)); // 零重力，纯弹簧动力学
    float dt = 1.0f / 60.0f;

    Body* anchor = new Body(new Box(1.0f, 1.0f), 0.0f, 0.0f, 0.0f); // 静态锚点
    world.addBody(anchor);
    Body* ball = new Body(new Box(1.0f, 1.0f), 7.0f, 0.0f, 1.0f);   // 1kg，拉长 2 米
    ball->setSleepAllow(false);
    world.addBody(ball);

    DistanceJointDef def;
    def.bodyA = anchor;
    def.bodyB = ball;
    def.length = 5.0f;        // 平衡杆长
    def.frequencyHz = 2.0f;   // 周期 0.5s = 30 帧
    def.dampingRatio = 0.0f;  // 无阻尼
    world.createJoint(def);

    // 理论弹簧刚度与初始机械能: k = m·(2πf)², E0 = ½k·A² (A=2)
    float kSpring = 1.0f * 4.0f * Settings::PAI * Settings::PAI * 4.0f;
    float E0 = 0.5f * kSpring * 2.0f * 2.0f;

    float maxX = -1e9f, minX = 1e9f;
    float maxEnergyDeviation = 0.0f;
    int zeroCrossings = 0;
    bool wasAbove = true; // 初始 x=7 在平衡点右侧
    for (int i = 0; i < 120; ++i) {
        world.step(dt);
        float x = ball->getPosition().getX();
        float v = ball->getVelocity().getX();
        maxX = std::max(maxX, x);
        minX = std::min(minX, x);

        // 机械能: E = ½mv² + ½k·Δx²
        float energy = 0.5f * v * v + 0.5f * kSpring * (x - 5.0f) * (x - 5.0f);
        maxEnergyDeviation = std::max(maxEnergyDeviation, std::abs(energy - E0) / E0);

        // 简谐往复计数（过平衡点，向上穿越）
        bool above = (x > 5.0f + 0.05f);
        if (above != wasAbove) {
            if (above) zeroCrossings++; // 从下往上穿过
            wasAbove = above;
        }
    }

    // 振幅严格保持 2 米（无衰减）、能量有界振荡（辛欧拉影子哈密顿量，无长期漂移）、
    // 持续往复（120 帧 = 4 个周期，向上穿越 4 次）
    bool ok = (maxX > 6.99f) && (minX < 3.01f)
        && (maxEnergyDeviation < 0.15f) // 辛格式：能量在 ±~11% 带内有界振荡，绝不衰减/发散
        && (zeroCrossings >= 3);

    Logger::info("Harmonic: maxX=" + std::to_string(maxX) +
        " minX=" + std::to_string(minX) +
        " energyDev=" + std::to_string(maxEnergyDeviation * 100.0f) + "%" +
        " crossings=" + std::to_string(zeroCrossings) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 2: 临界阻尼吸能测试 (Critical Damping) ---
// frequencyHz=4.0、dampingRatio=1.0，拉长 2 米释放，过冲必须 < 0.05 米
bool RunCriticalDampingTest() {
    World world(Vector2(0, 0));
    float dt = 1.0f / 60.0f;

    Body* anchor = new Body(new Box(1.0f, 1.0f), 0.0f, 0.0f, 0.0f);
    world.addBody(anchor);
    Body* ball = new Body(new Box(1.0f, 1.0f), 7.0f, 0.0f, 1.0f);
    ball->setSleepAllow(false);
    world.addBody(ball);

    DistanceJointDef def;
    def.bodyA = anchor;
    def.bodyB = ball;
    def.length = 5.0f;
    def.frequencyHz = 4.0f;
    def.dampingRatio = 1.0f; // 临界阻尼
    world.createJoint(def);

    float maxOvershoot = 0.0f;
    for (int i = 0; i < 120; ++i) {
        world.step(dt);
        float x = ball->getPosition().getX();
        if (x < 5.0f) {
            maxOvershoot = std::max(maxOvershoot, 5.0f - x); // 越过平衡点的深度
        }
    }

    float finalError = std::abs(ball->getPosition().getX() - 5.0f);
    bool ok = (maxOvershoot < 0.05f) && (finalError < 0.01f);

    Logger::info("CriticalDamping: overshoot=" + std::to_string(maxOvershoot) +
        " finalError=" + std::to_string(finalError) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

// --- 场景 3: 极端高刚度防发散测试 (Extreme Stiffness Stress Test) ---
// frequencyHz=60.0（ω·dt ≈ 6.28，显式欧拉必爆），挂 10kg 重物 300 帧
bool RunExtremeStiffnessTest() {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* ceiling = new Body(new Box(2.0f, 1.0f), 0.0f, 10.0f, 0.0f);
    world.addBody(ceiling);
    Body* weight = new Body(new Box(1.0f, 1.0f), 0.0f, 5.0f, 10.0f); // 10kg 重物
    weight->setSleepAllow(false);
    world.addBody(weight);

    DistanceJointDef def;
    def.bodyA = ceiling;
    def.bodyB = weight;
    def.length = 5.0f;
    def.frequencyHz = 60.0f;   // 极端坚硬
    def.dampingRatio = 1.0f;
    world.createJoint(def);

    float maxAbsV = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        Vector2 v = weight->getVelocity();
        float speed = std::abs(v.getX()) + std::abs(v.getY());
        maxAbsV = std::max(maxAbsV, speed);
        if (std::isnan(weight->getPosition().getX()) || std::isnan(weight->getPosition().getY())
            || std::isnan(v.getX()) || std::isnan(v.getY())) {
            hasNaN = true;
            break;
        }
    }

    float dist = (weight->getPosition() - ceiling->getPosition()).length();
    // 无 NaN、速度有界（隐式欧拉绝对稳定）、重物平稳悬挂在杆长附近
    bool ok = !hasNaN && (maxAbsV < 50.0f) && std::abs(dist - 5.0f) < 0.05f;

    Logger::info("ExtremeStiffness: maxSpeed=" + std::to_string(maxAbsV) +
        " dist=" + std::to_string(dist) +
        " NaN=" + std::to_string(hasNaN) +
        " -> " + std::string(ok ? "PASS" : "FAIL"));
    return ok;
}

//int main() {
//    Logger::info(">>> Starting V3 009: Spring Test...");
//    bool ok = true;
//    ok &= RunHarmonicTest();
//    ok &= RunCriticalDampingTest();
//    ok &= RunExtremeStiffnessTest();
//    Logger::info(ok ? ">>> ALL TESTS PASSED <<<" : ">>> SOME TESTS FAILED <<<");
//    return ok ? 0 : 1;
//}
