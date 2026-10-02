// GoogleTest 版 V3/010：RevoluteJoint 基础测试
// 移植自 tests/V3/010.cpp，场景逻辑原样保留，断言改为 gtest 宏
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RevoluteJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Logger.h"
#include <cmath>

// --- 场景 1: 绝对定点销钉旋转测试 (Fixed Pin Rotation) ---
// 静态天花板 A(0,5)，长条 B 左端钉在 (0,5)，水平释放后绕钉子摆动 300 帧
TEST(V3_010, FixedPinRotation) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* pin = new Body(new Box(1.0f, 1.0f), 0.0f, 5.0f, 0.0f); // 静态销钉
    // 销钉盒与长条在锚点处物理重叠：若参与实体碰撞，睡眠系统会把长条速度
    // 每帧清零（hasSolidContact），长条被冻结在出生姿态。设为触发器=纯锚点载体
    pin->getShape()->isTrigger = true;
    world.addBody(pin);
    // 长条矩形，左端锚点 (-2,0)。水平释放时质心与销钉同高、重力矩严格为零
    // （不稳定平衡，数值上不会倒下），所以给 0.5 rad 的初始倾斜；
    // 摆放位置按"绕锚点旋转"计算，保证出生瞬间锚点与销钉精确重合
    float tilt = 0.5f;
    Vector2 rB(-2.0f, 0.0f);
    rB = rB.rotate(tilt);
    Body* plank = new Body(new Box(4.0f, 0.5f), -rB.getX(), 5.0f - rB.getY(), 1.0f);
    plank->setSleepAllow(false);
    plank->setRotation(tilt);
    world.addBody(plank);

    RevoluteJointDef def;
    def.bodyA = pin;
    def.bodyB = plank;
    def.localAnchorA = Vector2(0.0f, 0.0f);   // 钉在销钉中心
    def.localAnchorB = Vector2(-2.0f, 0.0f);  // 长条左端
    RevoluteJoint* joint = static_cast<RevoluteJoint*>(world.createJoint(def));

    float maxAnchorError = 0.0f;
    float maxRadiusError = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        // 锚点重合误差：肉眼零脱臼的标准
        float anchorErr = (joint->getAnchorB() - joint->getAnchorA()).length();
        maxAnchorError = std::max(maxAnchorError, anchorErr);
        // 质心绕 (0,5) 做圆周运动：半径恒为 2
        float r = (plank->getPosition() - Vector2(0.0f, 5.0f)).length();
        maxRadiusError = std::max(maxRadiusError, std::abs(r - 2.0f));
    }

    EXPECT_LT(maxAnchorError, 1e-4f);
    EXPECT_LT(maxRadiusError, 0.02f);
}

// --- 场景 2: 两动态物体自由铰链测试 (Free Hinge Mechanics) ---
// 两个动态矩形角点对角点钉在一起下落撞地，铰链必须牢牢咬合（像剪刀折叠），绝不脱开
TEST(V3_010, FreeHingeMechanics) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.addBody(new Body(new Box(30.0f, 2.0f), 0.0f, 0.0f, 0.0f)); // 地面（顶面 y=1）

    Body* A = new Body(new Box(2.0f, 1.0f), 0.0f, 5.0f, 1.0f); // 动态矩形 A
    world.addBody(A);
    Body* B = new Body(new Box(2.0f, 1.0f), 2.0f, 4.0f, 1.0f); // 动态矩形 B
    world.addBody(B);

    // 角点对角点：A 的右下角 (1,-0.5) 与 B 的左上角 (-1,0.5) 在世界 (1,4.5) 处重合
    RevoluteJointDef def;
    def.bodyA = A;
    def.bodyB = B;
    def.localAnchorA = Vector2(1.0f, -0.5f);
    def.localAnchorB = Vector2(-1.0f, 0.5f);
    RevoluteJoint* joint = static_cast<RevoluteJoint*>(world.createJoint(def));

    float maxAnchorError = 0.0f;
    for (int i = 0; i < 240; ++i) {
        world.step(dt);
        float anchorErr = (joint->getAnchorB() - joint->getAnchorA()).length();
        maxAnchorError = std::max(maxAnchorError, anchorErr);
    }

    // 撞地后铰链依然咬合（误差全程可控），且两个物体都停留在地面附近没有飞走
    EXPECT_LT(maxAnchorError, 0.05f);
    EXPECT_LT(A->getPosition().getY(), 3.0f);
    EXPECT_LT(B->getPosition().getY(), 3.0f);
}

// --- 场景 3: 大质量比抗撕扯测试 (Mass Ratio Stress Test) ---
// 100kg 大铁球与 1kg 小木棍铰接，落撞击地；8 次迭代下锚点必须牢固锁死、数值平稳
TEST(V3_010, MassRatioStress) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.addBody(new Body(new Box(30.0f, 2.0f), 0.0f, 0.0f, 0.0f)); // 地面（顶面 y=1）

    // 100kg 大铁球：半径 1，密度 = 100/π
    float ballDensity = 100.0f / Settings::PAI;
    Body* ball = new Body(new Circle(1.0f), 0.0f, 5.0f, ballDensity);
    world.addBody(ball);

    // 1kg 小木棍：2×0.2，密度 2.5；铰接在球右边缘 (1,0)——避免与球体重叠
    // （重叠会产生实体接触，睡眠系统会把两者速度清零，整个系统被冻结）
    Body* stick = new Body(new Box(2.0f, 0.2f), 2.1f, 5.0f, 2.5f);
    world.addBody(stick);

    // 木棍左端铰接在球的右边缘
    RevoluteJointDef def;
    def.bodyA = ball;
    def.bodyB = stick;
    def.localAnchorA = Vector2(1.0f, 0.0f);
    def.localAnchorB = Vector2(-1.0f, 0.0f);
    RevoluteJoint* joint = static_cast<RevoluteJoint*>(world.createJoint(def));

    float maxAnchorError = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        float anchorErr = (joint->getAnchorB() - joint->getAnchorA()).length();
        maxAnchorError = std::max(maxAnchorError, anchorErr);
        if (std::isnan(stick->getPosition().getX()) || std::isnan(stick->getPosition().getY())) {
            hasNaN = true;
            break;
        }
    }

    // 100:1 质量比下锚点依然锁死、无 NaN、无抖动发散；球真实落地（y≈2）
    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxAnchorError, 0.01f);
    EXPECT_LT(ball->getPosition().getY(), 3.0f);
}
