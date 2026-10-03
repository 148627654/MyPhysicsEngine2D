// ============================================================
// 2Ddemo 样例 3：悬索大吊桥 (Suspension Bridge)
//   - 2 个静态桥塔（触发器：纯锚点载体）+ 12 块木板 Box
//   - 13 个 DistanceJoint：桥塔↔端板刚性吊杆 + 木板之间首尾相接（length=0 销接）
//   - 整桥在重力下呈悬链线下垂；小球从左侧滚过桥面，验证多体张力传递
// 运行：取消注释本文件 main，注释其他文件（tests 与 2Ddemo）的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "DistanceJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Logger.h"
#include <cmath>
#include <vector>

// 装配一根距离关节并返回（用于监控锚点误差）
static DistanceJoint* MakeRod(World& world, Body* a, Body* b,
    const Vector2& localA, const Vector2& localB, float length) {
    DistanceJointDef def;
    def.bodyA = a;
    def.bodyB = b;
    def.collideConnected = false;
    def.localAnchorA = localA;
    def.localAnchorB = localB;
    def.length = length; // 刚性杆（frequencyHz=0）
    return static_cast<DistanceJoint*>(world.createJoint(def));
}

//int main() {
    //Logger::info(">>> 2Ddemo 3: Bridge 悬索吊桥（12 木板 + 13 距离关节）<<<");
    //World world(Vector2(0.0f, -9.8f));
    //float dt = 1.0f / 60.0f;

    //const int N = 12;
    //const float PLANK_W = 1.1f;      // 木板宽
    //const float PLANK_H = 0.15f;     // 木板厚
    //const float BRIDGE_Y = 5.0f;     // 桥面高度

    //// --- 两端静态桥塔（触发器：纯锚点载体，避免与端板接触冻结）---
    //// 【设计要点】桥塔锚点必须落在端板边缘的内侧：链长（13.6）> 跨度（12.8），
    //// 桥才能在重力下形成悬链线下垂；若链长 ≤ 跨度则链条根本挂不住（整体坠落）
    //Body* towerL = new Body(new Box(0.6f, 1.4f), -6.7f, 4.6f, 0.0f);
    //towerL->getShape()->isTrigger = true;
    //world.addBody(towerL);
    //Body* towerR = new Body(new Box(0.6f, 1.4f), 6.7f, 4.6f, 0.0f);
    //towerR->getShape()->isTrigger = true;
    //world.addBody(towerR);

    //// --- 12 块木板：首尾相接（中心间距 = 板宽，边与边正好贴合）---
    //// 【预弯成悬链线形】直线初态的摆动能无阻尼持续振荡（刚体链没有空气阻力），
    //// 预弯成抛物线近似静态平衡，桥出生即安稳，只有小球滚过时的轻微张力波
    //Body* planks[N];
    //for (int i = 0; i < N; ++i) {
        //float x = -6.05f + i * PLANK_W; // 板 0 中心 -6.05，板 11 中心 +6.05
        //float u = 2.0f * i / (N - 1) - 1.0f;          // -1 ~ +1
        //float y = BRIDGE_Y - 3.5f * (1.0f - u * u);   // 抛物线预弯：中板下垂 3.5
        //planks[i] = new Body(new Box(PLANK_W, PLANK_H), x, y, 1.0f);
        //planks[i]->setSleepAllow(false);
        //world.addBody(planks[i]);
    //}

    //// --- 13 个距离关节 ---
    //std::vector<DistanceJoint*> rods;
    //// 桥塔 ↔ 端板（刚性吊杆：塔面 (±6.4,5.0) 到端板边缘 (±6.6,5.0)，长 0.2）
    //rods.push_back(MakeRod(world, towerL, planks[0],
        //Vector2(0.3f, 0.4f), Vector2(-0.55f, 0.0f), 0.2f));
    //rods.push_back(MakeRod(world, towerR, planks[N - 1],
        //Vector2(-0.3f, 0.4f), Vector2(0.55f, 0.0f), 0.2f));
    //// 木板之间（length=0 弹性销接：软弹簧模式 4Hz / 阻尼比 0.7）
    //// 刚性链无阻尼，摆动能不衰减，拉伸随摆动张力周期性振荡永不停息；
    //// 软弹簧让链条"弹性拉扯下平滑起伏"（规格原文），并衰减沉降成稳定悬链线
    //for (int i = 0; i < N - 1; ++i) {
        //DistanceJointDef def;
        //def.bodyA = planks[i];
        //def.bodyB = planks[i + 1];
        //def.collideConnected = false;
        //def.localAnchorA = Vector2(0.55f, 0.0f);
        //def.localAnchorB = Vector2(-0.55f, 0.0f);
        //def.length = 0.0f;
        //def.frequencyHz = 4.0f;
        //def.dampingRatio = 0.7f;
        //rods.push_back(static_cast<DistanceJoint*>(world.createJoint(def)));
    //}
    //Logger::info("组装完成: 2 桥塔 + " + std::to_string(N) + " 木板 + " +
        //std::to_string(rods.size()) + " 距离关节");

    //// --- 小球从左侧滚上桥面：初速需足够大才能滚过桥谷（需克服 3.5m 下垂的势能
    //// √(2gh) ≈ 8.3 m/s），给 10 m/s 留裕量
    //Body* ball = new Body(new Circle(0.3f), -6.0f, 6.3f, 1.0f);
    //ball->getShape()->material.dynamicFriction = 0.6f;
    //ball->getShape()->material.restitution = 0.2f;
    //ball->setVelocity(Vector2(10.0f, 0.0f));
    //ball->setSleepAllow(false);
    //world.addBody(ball);

    //// --- 模拟 10 秒 ---
    //float maxSag = 0.0f;
    //float maxAnchorError = 0.0f;
    //bool hasNaN = false;
    //for (int i = 0; i < 600; ++i) {
        //world.step(dt);

        //float sag = BRIDGE_Y - planks[N / 2]->getPosition().getY(); // 中板下坠量
        //maxSag = std::max(maxSag, sag);
        //for (DistanceJoint* r : rods) {
            //float err = (r->getAnchorB() - r->getAnchorA()).length();
            //maxAnchorError = std::max(maxAnchorError, err);
        //}
        //if (std::isnan(planks[N / 2]->getPosition().getY()) || std::isnan(ball->getPosition().getX())) {
            //hasNaN = true;
            //break;
        //}

        //if ((i + 1) % 60 == 0) {
            //Logger::info("t=" + std::to_string((i + 1) / 60) + "s" +
                //" 中板y=" + std::to_string(planks[N / 2]->getPosition().getY()) +
                //" 下垂=" + std::to_string(sag) +
                //" 球=(" + std::to_string(ball->getPosition().getX()) + "," +
                //std::to_string(ball->getPosition().getY()) + ")" +
                //" maxAnchorErr=" + std::to_string(maxAnchorError));
        //}
    //}

    //// --- 验收断言 ---
    //float settledSag = BRIDGE_Y - planks[N / 2]->getPosition().getY(); // 末帧静息下垂
    //bool sagOk = (settledSag > 1.5f) && (settledSag < 5.5f); // 悬链线下垂（预弯 3.5 ± 弹性伸长）
    //bool jointOk = (maxAnchorError < 1.5f);                // 弹性销接静载伸长 + 初始沉降瞬态（刚性桥塔吊杆静息 < 0.05）
    //bool ballOk = (ball->getPosition().getX() > 4.0f);     // 小球滚过桥面
    //Logger::info("验收: 静息下垂=" + std::to_string(settledSag) + "（全程最大 " + std::to_string(maxSag) + "）-> " +
        //std::string(sagOk ? "PASS" : "FAIL"));
    //Logger::info("验收: 关节最大锚点误差=" + std::to_string(maxAnchorError) + " -> " +
        //std::string(jointOk ? "PASS" : "FAIL"));
    //Logger::info("验收: 球x=" + std::to_string(ball->getPosition().getX()) +
        //"（滚过桥面: " + std::string(ballOk ? "PASS" : "FAIL") + "）");
    //Logger::info(std::string(">>> BridgeDemo ") +
        //((sagOk && jointOk && ballOk && !hasNaN) ? "PASS" : "FAIL") + " <<<");
    //return (sagOk && jointOk && ballOk && !hasNaN) ? 0 : 1;
//}
