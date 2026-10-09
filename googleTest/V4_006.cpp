// GoogleTest 版 V4/006：GearJoint 齿轮关节测试
// 场景 1 1:1 外齿轮反向啮合：齿轮 A 马达恒速 +6 rad/s，B 被齿轮关节强迫为
//   -6.000000 rad/s（反转、严格等速），转速比误差 < 1e-5
//   【物理注记】一次性初速度会被动量守恒均分（B 只会 -3）；要 B=-6 必须 A 持续驱动
// 场景 2 1:3 减速变速箱：小轮 +12 马达驱动，大轮稳定 -4.000000 恒速，
//   传动比误差严格为零、稳态机械能恒定
// 场景 3 齿轮齿条机构：齿轮(Revolute) + 齿条(水平 Prismatic) 耦合，
//   齿条纯线性前进，位移严格满足 Δx = -radius·Δθ
// 场景 4 级联安全自毁：销毁父关节 → GearJoint 自动级联销毁，后续模拟无野指针崩溃
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "GearJoint.h"
#include "RevoluteJoint.h"
#include "PrismaticJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Setting.h"
#include <cmath>
#include <algorithm>

namespace {
// 天花板 + 两个半径 1 齿轮的装配（测试 1/2/4 共用）：返回两个销钉关节
struct GearPairScene {
    World world{ Vector2(0, -9.8f) };
    Body* gearA;
    Body* gearB;
    RevoluteJoint* revA;
    RevoluteJoint* revB;

    GearPairScene() {
        Body* ceiling = new Body(new Box(3.0f, 0.5f), 0.0f, 3.0f, 0.0f); // 静态天花板
        world.addBody(ceiling);

        // 两个半径 1 齿轮（质量 2π ≈ 6.28），钉在天花板下、互不接触（中心距 3）
        float density = 2.0f;
        gearA = new Body(new Circle(1.0f), -1.5f, 1.5f, density);
        gearB = new Body(new Circle(1.0f), 1.5f, 1.5f, density);
        world.addBody(gearA);
        world.addBody(gearB);

        RevoluteJointDef rA;
        rA.bodyA = ceiling;
        rA.bodyB = gearA;
        rA.localAnchorA = Vector2(-1.5f, -1.5f);
        rA.localAnchorB = Vector2(0.0f, 0.0f);
        rA.collideConnected = false;
        revA = static_cast<RevoluteJoint*>(world.createJoint(rA));

        RevoluteJointDef rB;
        rB.bodyA = ceiling;
        rB.bodyB = gearB;
        rB.localAnchorA = Vector2(1.5f, -1.5f);
        rB.localAnchorB = Vector2(0.0f, 0.0f);
        rB.collideConnected = false;
        revB = static_cast<RevoluteJoint*>(world.createJoint(rB));
    }

    // 给齿轮 A 装载恒速马达（"施加角速度"= 持续驱动）
    void driveA(float speed) {
        revA->setMotorSpeed(speed);
        revA->setMaxMotorTorque(1000.0f);
        revA->enableMotor(true);
    }
};
} // namespace

// --- 场景 1: 1:1 外齿轮反向啮合测试 (1:1 Meshing External Gears) ---
TEST(V4_006, MeshingExternalGears1to1) {
    GearPairScene s;
    s.driveA(6.0f); // A 恒速 +6 rad/s（逆时针）
    float dt = 1.0f / 60.0f;

    GearJointDef gdef;
    gdef.joint1 = s.revA;
    gdef.joint2 = s.revB;
    gdef.ratio = 1.0f;
    GearJoint* gear = static_cast<GearJoint*>(s.world.createJoint(gdef));
    ASSERT_NE(gear, nullptr);

    for (int i = 0; i < 120; ++i) s.world.step(dt);

    float wA = s.gearA->getAngularVelocity();
    float wB = s.gearB->getAngularVelocity();
    EXPECT_NEAR(wA, 6.0f, 1e-4f);         // A 保持马达驱动
    EXPECT_NEAR(wB, -6.0f, 1e-4f);        // B 被瞬间强迫为 -6.000000（顺时针反转）
    EXPECT_LT(std::abs(wA + wB), 1e-5f);  // 转速比误差 < 1e-5（严格相等）
}

// --- 场景 2: 1:3 减速变速箱机械测试 (1:3 Speed Reduction Ratio) ---
TEST(V4_006, SpeedReduction1to3) {
    GearPairScene s;
    s.driveA(12.0f); // 小轮恒速 +12 rad/s
    float dt = 1.0f / 60.0f;

    GearJointDef gdef;
    gdef.joint1 = s.revA;
    gdef.joint2 = s.revB;
    gdef.ratio = 3.0f; // 3:1 减速：wA + 3·wB = 0 → wB = -4
    GearJoint* gear = static_cast<GearJoint*>(s.world.createJoint(gdef));
    ASSERT_NE(gear, nullptr);

    float maxRatioError = 0.0f;
    float keAtFrame60 = 0.0f;
    float keAtEnd = 0.0f;
    float inertia = 3.14159f; // Circle(1) 密度 2 → I = m·r²/2 ≈ π
    for (int i = 0; i < 300; ++i) {
        s.world.step(dt);
        float wA = s.gearA->getAngularVelocity();
        float wB = s.gearB->getAngularVelocity();
        maxRatioError = std::max(maxRatioError, std::abs(wA + 3.0f * wB));
        float ke = 0.5f * inertia * (wA * wA + wB * wB);
        if (i == 60) keAtFrame60 = ke;
        if (i == 299) keAtEnd = ke;
    }

    float wA = s.gearA->getAngularVelocity();
    float wB = s.gearB->getAngularVelocity();
    EXPECT_NEAR(wA, 12.0f, 1e-4f);       // 小轮恒速
    EXPECT_NEAR(wB, -4.0f, 1e-4f);       // 大轮稳定 -4.000000（3:1 减速）
    EXPECT_LT(maxRatioError, 1e-5f);     // 传动比误差严格为零（全程）
    // 机械能守恒：稳态后动能恒定（马达匀速、齿轮无耗散）
    EXPECT_LT(std::abs(keAtEnd - keAtFrame60), 1e-3f);
}

// --- 场景 3: 齿轮齿条机构线性推进测试 (Rack and Pinion) ---
// 齿轮半径 0.5，GearJoint ratio = 1/radius = 2 → 约束 θ + 2·x = 0：
// Δx = -Δθ/2 = -radius·Δθ，齿条随齿轮旋转平稳做纯线性前进
TEST(V4_006, RackAndPinion) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    Body* pin = new Body(new Box(0.4f, 0.4f), 0.0f, 1.5f, 0.0f);   // 静态销钉
    world.addBody(pin);
    Body* rail = new Body(new Box(0.4f, 0.4f), 0.0f, -0.5f, 0.0f); // 静态导轨
    world.addBody(rail);

    // 齿轮：半径 0.5、质量 1，钉在 (0,1.5)
    float gearDensity = 1.0f / (Settings::PAI * 0.25f);
    Body* gear = new Body(new Circle(0.5f), 0.0f, 1.5f, gearDensity);
    world.addBody(gear);

    // 齿条：2×0.5 箱、质量 2，水平滑轨支撑。
    // 出生点必须在导轨锚点高度 (0,-0.5)——否则滑块垂向约束会把它拉到锚点线上
    Body* rack = new Body(new Box(2.0f, 0.5f), 0.0f, -0.5f, 2.0f);
    world.addBody(rack);

    RevoluteJointDef rDef;
    rDef.bodyA = pin;
    rDef.bodyB = gear;
    rDef.localAnchorA = Vector2(0.0f, 0.0f);
    rDef.localAnchorB = Vector2(0.0f, 0.0f);
    rDef.enableMotor = true;
    rDef.motorSpeed = -3.0f;    // 齿轮恒速 -3 rad/s
    rDef.maxMotorTorque = 100.0f;
    RevoluteJoint* rev = static_cast<RevoluteJoint*>(world.createJoint(rDef));
    ASSERT_NE(rev, nullptr);

    PrismaticJointDef pDef;
    pDef.bodyA = rail;
    pDef.bodyB = rack;
    pDef.localAnchorA = Vector2(0.0f, 0.0f);
    pDef.localAnchorB = Vector2(0.0f, 0.0f);
    pDef.localAxisA = Vector2(1.0f, 0.0f); // 水平滑轨
    pDef.collideConnected = false;
    PrismaticJoint* pris = static_cast<PrismaticJoint*>(world.createJoint(pDef));
    ASSERT_NE(pris, nullptr);

    GearJointDef gDef;
    gDef.joint1 = rev;   // Revolute：θ
    gDef.joint2 = pris;  // Prismatic：x
    gDef.ratio = 2.0f;   // = 1/radius（半径 0.5）
    GearJoint* gearJoint = static_cast<GearJoint*>(world.createJoint(gDef));
    ASSERT_NE(gearJoint, nullptr);

    float maxIdentityError = 0.0f;
    for (int i = 0; i < 240; ++i) {
        world.step(dt);
        float identity = std::abs(gear->getRotation() + 2.0f * rack->getPosition().getX());
        maxIdentityError = std::max(maxIdentityError, identity);
    }

    // 4 秒后：θ ≈ -12 rad → 齿条前进 ≈ +6 m，位移严格满足 Δx = -radius·Δθ
    // （θ 与 -12 差 ~0.05：启动首帧齿轮位置修正把初始角位移的一部分重分配给了
    //  齿条以建立正确传动比——这正是耦合约束的物理，恒等式才是硬断言）
    float deltaTheta = gear->getRotation();
    float deltaX = rack->getPosition().getX();
    EXPECT_NEAR(deltaTheta, -12.0f, 0.15f);          // 马达巡航（含启动重分配）
    EXPECT_NEAR(deltaX, -0.5f * deltaTheta, 1e-4f);  // 传动比位移严格满足
    EXPECT_LT(maxIdentityError, 1e-4f);              // 耦合恒等式全程保持
    // 纯线性前进：不旋转、无纵向漂移（出生在导轨锚点高度 -0.5）
    EXPECT_LT(std::abs(rack->getRotation()), 1e-4f);
    EXPECT_LT(std::abs(rack->getPosition().getY() - (-0.5f)), 1e-4f);
}

// --- 场景 4: 级联安全自毁测试 (Cascade Invalidation) ---
TEST(V4_006, CascadeInvalidation) {
    GearPairScene s;
    float dt = 1.0f / 60.0f;

    GearJointDef gdef;
    gdef.joint1 = s.revA;
    gdef.joint2 = s.revB;
    gdef.ratio = 1.0f;
    GearJoint* gear = static_cast<GearJoint*>(s.world.createJoint(gdef));
    ASSERT_NE(gear, nullptr);
    EXPECT_EQ(s.world.getJointCount(), 3); // 2 销钉 + 1 齿轮

    // 销毁父关节 revA → 齿轮必须自动级联销毁（否则齿轮持有悬空野指针）
    s.world.destroyJoint(s.revA);
    EXPECT_EQ(s.world.getJointCount(), 1); // 只剩 revB

    // 再销毁 revB：世界关节清空
    s.world.destroyJoint(s.revB);
    EXPECT_EQ(s.world.getJointCount(), 0);

    // 后续模拟 300 帧：若齿轮没被级联销毁，会在求解时解引用野指针而崩溃
    for (int i = 0; i < 300; ++i) s.world.step(dt);
    SUCCEED(); // 跑完没崩即通过
}
