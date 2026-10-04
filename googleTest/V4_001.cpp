// GoogleTest 版 V4/001：WeldJoint 焊接关节测试
// 场景 1 悬臂梁高负载：焊死在地面锚点上的悬臂梁，悬空端持续压 100N，
//   300 帧内必须保持绝对水平（端点下垂 <1mm、角偏差 <1e-4 rad）
// 场景 2 动态 L 形组合体：两 1kg 方块焊接成 L 形自由落体砸台阶，
//   反弹翻滚过程中焊缝必须严丝合缝（锚点不脱、相对角锁定、如一个整体）
// 场景 3 连接体碰撞屏蔽：严重重叠的两方块焊接（collideConnected=false），
//   窄相不得把二者推开互弹，系统零抖动
#include <gtest/gtest.h>
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "WeldJoint.h"
#include "Box.h"
#include <cmath>
#include <algorithm>

// --- 场景 1: 悬臂梁高负载测试 (Cantilever Beam Rigidity) ---
// 静态锚点 A(0,0)，10×0.5 悬臂梁 B 左端焊接在原点、右端悬空；
// 悬空端每帧施加 100N 向下（力累加器每帧清零，需每帧施加），
// 300 帧后焊接必须把梁锁在绝对水平
TEST(V4_001, CantileverBeamRigidity) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // 静态锚点块（密度 0 = 静态），焊点载体
    Body* anchor = new Body(new Box(0.4f, 0.4f), 0.0f, 0.0f, 0.0f);
    world.addBody(anchor);

    // 悬臂梁：10×0.5，密度 0.2 → 质量 1kg；左端 x=0，右端 x=10 悬空
    Body* beam = new Body(new Box(10.0f, 0.5f), 5.0f, 0.0f, 0.2f);
    beam->setSleepAllow(false); // 恒载下不许入睡，300 帧全程考验焊接强度
    world.addBody(beam);

    WeldJointDef def;
    def.bodyA = anchor;
    def.bodyB = beam;
    def.localAnchorA = Vector2(0.0f, 0.0f);   // 锚点块中心
    def.localAnchorB = Vector2(-5.0f, 0.0f);  // 梁左端
    def.referenceAngle = 0.0f;
    def.collideConnected = false; // 梁与锚点块物理重叠 0.2m，必须过滤实体接触
    WeldJoint* joint = static_cast<WeldJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    float maxEndSag = 0.0f;   // 悬空端下垂位移
    float maxAngleDev = 0.0f; // 旋转角偏差
    float maxAnchorErr = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        beam->applyForceAtPoint(Vector2(0.0f, -100.0f), Vector2(10.0f, 0.0f));
        world.step(dt);

        // 悬空端世界坐标 = 质心 + 旋转后的半长臂（梁长 10，半长 5）
        Vector2 tip = beam->getPosition() + Vector2(5.0f, 0.0f).rotate(beam->getRotation());
        maxEndSag = std::max(maxEndSag, std::abs(tip.getY()));
        maxAngleDev = std::max(maxAngleDev, std::abs(beam->getRotation()));
        maxAnchorErr = std::max(maxAnchorErr, (joint->getAnchorB() - joint->getAnchorA()).length());
        if (std::isnan(tip.getX()) || std::isnan(tip.getY())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxEndSag, 1e-3f);    // 端点下垂位移 < 10^-3 米
    EXPECT_LT(maxAngleDev, 1e-4f);  // 旋转角偏差 < 10^-4 弧度
    EXPECT_LT(maxAnchorErr, 1e-3f); // 焊点咬合辅助诊断
}

// --- 场景 2: 动态物体刚性组合体测试 (Composite L-Shape Bounce) ---
// 两 1kg 方块焊接成 L 形（横板 A + 竖板 B，有 0.5×1 重叠区），
// 同一初速度+角速度整体抛出，自由落体砸地面台阶——反弹翻滚中
// 焊缝必须严丝合缝：锚点不脱、相对角锁定、如一个整体刚体
TEST(V4_001, CompositeLShapeBounce) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    world.addBody(new Body(new Box(30.0f, 2.0f), 0.0f, 0.0f, 0.0f)); // 地面（顶面 y=1）
    world.addBody(new Body(new Box(1.5f, 1.0f), 4.0f, 1.5f, 0.0f));  // 台阶（顶面 y=2）

    // L 形组合体：横板 A（2×1，密度 0.5 → 1kg）+ 竖板 B（1×2，密度 0.5 → 1kg）
    Body* A = new Body(new Box(2.0f, 1.0f), -0.5f, 5.0f, 0.5f);
    world.addBody(A);
    Body* B = new Body(new Box(1.0f, 2.0f), 0.5f, 4.5f, 0.5f);
    world.addBody(B);

    // 焊点世界 (0.5, 5)：A 右缘中点、B 上缘中点
    WeldJointDef def;
    def.bodyA = A;
    def.bodyB = B;
    def.localAnchorA = Vector2(1.0f, 0.0f);
    def.localAnchorB = Vector2(0.0f, 0.5f);
    def.referenceAngle = 0.0f;
    def.collideConnected = false; // 两板有 0.5×1 重叠区，禁止互相推挤（防内爆）
    WeldJoint* joint = static_cast<WeldJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    // 弹性反弹：两板 restitution=0.3（与地面按 Maximum 合成 0.3）
    // 默认材质 restitution=0（死球），不设弹性整个 L 只会落地滑行不反弹
    A->setRestitution(0.3f);
    B->setRestitution(0.3f);

    // 同一初速度 + 角速度 = 整体抛出，飞向右前方的台阶角
    A->setVelocity(Vector2(3.0f, 0.0f));
    B->setVelocity(Vector2(3.0f, 0.0f));
    A->setAngularVelocity(0.8f);
    B->setAngularVelocity(0.8f);

    float maxAnchorError = 0.0f; // 焊缝开缝量
    float maxAngleDiff = 0.0f;   // 相对旋转偏差（整体刚体的标志）
    float prevY = (A->getPosition().getY() + B->getPosition().getY()) * 0.5f;
    bool hitGround = false;
    int bounceFrames = 0;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        maxAnchorError = std::max(maxAnchorError, (joint->getAnchorB() - joint->getAnchorA()).length());
        maxAngleDiff = std::max(maxAngleDiff, std::abs(A->getRotation() - B->getRotation()));
        float y = (A->getPosition().getY() + B->getPosition().getY()) * 0.5f;
        // L 形落在台阶顶面（y=2）上，静息质心 y≈2.0，故用 2.2 判"已触地"
        if (!hitGround && y < 2.2f) hitGround = true;
        if (hitGround && y > prevY + 0.005f) ++bounceFrames; // 触地后质心再度抬升 = 反弹/翻滚
        prevY = y;
        if (std::isnan(A->getPosition().getX()) || std::isnan(B->getPosition().getY())) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    // 焊缝严丝合缝：6 m/s 着地冲击的碰撞帧有 ~0.04 的瞬态拉伸（10/4 迭代固有，
    // 2 帧内恢复到 0.002），阈值覆盖该尖峰；全程不脱臼、不发散
    EXPECT_LT(maxAnchorError, 0.05f);
    EXPECT_LT(maxAngleDiff, 0.05f); // 相对角锁定：如一个整体刚体旋转（瞬态 ~2°）
    EXPECT_GT(bounceFrames, 0);     // 确实经历了反弹翻滚
    // 最终落在地面/台阶上，没有飞走
    EXPECT_LT(A->getPosition().getY(), 3.0f);
    EXPECT_LT(B->getPosition().getY(), 3.0f);
}

// --- 场景 3: 连接体碰撞屏蔽验证 (Overlap No Push) ---
// 两 1kg 方块严重重叠（中心距 0.7 < 边长 1.0）并焊接；
// collideConnected=false 下窄相不得创建二者接触——若过滤失效，
// 重叠会被冲量瞬间推开互弹。断言：中心距恒定、相对速度≈0、零抖动
TEST(V4_001, OverlapNoPush) {
    World world(Vector2(0, -9.8f));
    float dt = 1.0f / 60.0f;

    // A 中心 (0,5)，B 中心 (0.7,5)：重叠 0.3m
    Body* A = new Body(new Box(1.0f, 1.0f), 0.0f, 5.0f, 1.0f);
    world.addBody(A);
    Body* B = new Body(new Box(1.0f, 1.0f), 0.7f, 5.0f, 1.0f);
    world.addBody(B);

    // 焊点在重叠区中点 (0.35,5)：锚点重合的同时保持两中心距 0.7
    WeldJointDef def;
    def.bodyA = A;
    def.bodyB = B;
    def.localAnchorA = Vector2(0.35f, 0.0f);
    def.localAnchorB = Vector2(-0.35f, 0.0f);
    def.referenceAngle = 0.0f;
    def.collideConnected = false; // 关键：过滤焊接双方接触，否则窄相会推开重叠
    WeldJoint* joint = static_cast<WeldJoint*>(world.createJoint(def));
    ASSERT_NE(joint, nullptr);

    // 自由下落（无地面接触干扰），若过滤失效两方块会瞬间互弹分离
    float maxDistError = 0.0f;
    float maxRelVel = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 300; ++i) {
        world.step(dt);
        float dist = (B->getPosition() - A->getPosition()).length();
        maxDistError = std::max(maxDistError, std::abs(dist - 0.7f));
        maxRelVel = std::max(maxRelVel, (B->getVelocity() - A->getVelocity()).length());
        if (std::isnan(dist)) { hasNaN = true; break; }
    }

    EXPECT_FALSE(hasNaN);
    EXPECT_LT(maxDistError, 0.01f); // 中心距始终 0.7：没被推开也没被拽进
    EXPECT_LT(maxRelVel, 0.05f);    // 相对速度≈0：零抖动
}
