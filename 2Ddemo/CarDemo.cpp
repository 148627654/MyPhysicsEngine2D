// ============================================================
// 2Ddemo 样例 2：动力越野小车 (Suspension Car)
//   - 车身 Box + 悬挂副车架 Box + 2 个 Circle 车轮
//   - SpringJoint × 2（4Hz / 阻尼比 0.7）连接车身与副车架，吸收颠簸
//   - RevoluteJoint × 2 连接副车架与车轮，enableMotor 驱动（ω=20 rad/s）
//   - 凹凸路面（3 个减速带）→ 15° 上坡 → 高原平台
//   - 沿途 10 个金币触发器：验证连续 10 次 OnTriggerEnter 且无速度减损
//   - CSVExporter 导出全部刚体轨迹到 output/car_demo.csv
// 说明：物理上弹簧和销钉不能同时连接同一对刚体（销钉会把弹簧形变锁死），
//       所以引入悬挂副车架：弹簧挂车身，马达销钉挂车轮。
// 运行：取消注释本文件 main，注释其他文件（tests 与 2Ddemo）的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RevoluteJoint.h"
#include "SpringJoint.h"
#include "Box.h"
#include "Circle.h"
#include "ContactListener.h"
#include "CSVExporter.h"
#include "Logger.h"
#include <cmath>

// 触发器计数器：记录金币被吃次数
class CoinListener : public ContactListener {
public:
    int triggerEnter = 0;
    void onTriggerEnter(const TriggerEvent& event) override { triggerEnter++; }
};

//int main() {
    //Logger::info(">>> 2Ddemo 2: Car 避震小车（弹簧避震 + 马达驱动）<<<");
    //World world(Vector2(0.0f, -9.8f));
    //float dt = 1.0f / 60.0f;

    //// --- 赛道：平地 + 3 个减速带 + 15° 上坡 + 15° 下坡 + 高原平台 ---
    //// 减速带用倒 V 型平滑坡道（两块斜板拼成，按端点精确计算保证顶点干净对接）：
    //// 垂直墙车轮爬不上去（轮心被车轴钉住，会被墙卡死）；两板若重叠会形成
    //// 悬檐卡住车轮。斜板让车轮顺坡度滚过，弹簧吸收颠簸
    //world.addBody(new Body(new Box(40.0f, 1.0f), 0.0f, -0.5f, 0.0f));   // 平地（顶面 y=0，x∈[-20,20]，接坡道起点）
    //Body* bumpUp1 = new Body(new Box(1.8f, 0.15f), 3.107f, 0.186f, 0.0f); bumpUp1->setRotation(0.125f);  world.addBody(bumpUp1);
    //Body* bumpDn1 = new Body(new Box(1.8f, 0.15f), 4.893f, 0.186f, 0.0f); bumpDn1->setRotation(-0.125f); world.addBody(bumpDn1);  // 减速带 1（高 0.22）
    //Body* bumpUp2 = new Body(new Box(2.0f, 0.15f), 7.010f, 0.214f, 0.0f); bumpUp2->setRotation(0.14f);   world.addBody(bumpUp2);
    //Body* bumpDn2 = new Body(new Box(2.0f, 0.15f), 8.990f, 0.214f, 0.0f); bumpDn2->setRotation(-0.14f);  world.addBody(bumpDn2);  // 减速带 2（高 0.28）
    //Body* bumpUp3 = new Body(new Box(1.6f, 0.15f), 11.204f, 0.149f, 0.0f); bumpUp3->setRotation(0.094f); world.addBody(bumpUp3);
    //Body* bumpDn3 = new Body(new Box(1.6f, 0.15f), 12.796f, 0.149f, 0.0f); bumpDn3->setRotation(-0.094f); world.addBody(bumpDn3); // 减速带 3（高 0.15）
    //Body* ramp = new Body(new Box(26.0f, 0.5f), 32.5f, 3.4f, 0.0f);     // 15° 斜坡（x∈[20,45]）
    //ramp->setRotation(0.2618f);
    //world.addBody(ramp);
    //world.addBody(new Body(new Box(60.0f, 1.0f), 75.0f, 6.9f, 0.0f));   // 高原平台（顶面 y=7.4）

    //// --- 车轮（高摩擦防打滑）---
    //Body* wheelL = new Body(new Circle(0.5f), -0.9f, 0.5f, 1.0f);
    //Body* wheelR = new Body(new Circle(0.5f), 0.9f, 0.5f, 1.0f);
    //for (Body* w : { wheelL, wheelR }) {
        //w->getShape()->material.dynamicFriction = 1.0f;
        //w->getShape()->material.staticFriction = 1.0f;
        //w->setSleepAllow(false);
        //world.addBody(w);
    //}

    //// --- 悬挂副车架 + 车身 ---
    //Body* plate = new Body(new Box(2.2f, 0.25f), 0.0f, 1.55f, 1.5f);
    //plate->setSleepAllow(false);
    //world.addBody(plate);
    //Body* chassis = new Body(new Box(2.8f, 0.5f), 0.0f, 2.35f, 1.5f);
    //chassis->setSleepAllow(false);
    //world.addBody(chassis);

    //// --- 弹簧避震：车身 ↔ 副车架（软约束，4Hz / 0.7 临界偏下）---
    //SpringJoint* springL = nullptr, * springR = nullptr;
    //{
        //SpringJointDef def;
        //def.bodyA = chassis; def.bodyB = plate;
        //def.collideConnected = false;
        //def.localAnchorA = Vector2(-0.9f, -0.25f); // 世界 (-0.9, 2.1)
        //def.localAnchorB = Vector2(-0.9f, 0.125f); // 世界 (-0.9, 1.675)
        //def.length = 0.425f;
        //def.frequencyHz = 4.0f;
        //def.dampingRatio = 0.7f;
        //springL = static_cast<SpringJoint*>(world.createJoint(def));
        //def.localAnchorA = Vector2(0.9f, -0.25f);
        //def.localAnchorB = Vector2(0.9f, 0.125f);
        //springR = static_cast<SpringJoint*>(world.createJoint(def));
    //}

    //// --- 车轮马达：副车架 ↔ 车轮（负转速 = 顺时针 = 向右行驶）---
    //RevoluteJoint* axleL = nullptr, * axleR = nullptr;
    //{
        //RevoluteJointDef def;
        //def.collideConnected = false;
        //def.initialize(plate, wheelL, Vector2(-0.9f, 0.5f));
        //def.enableMotor = true;
        //def.motorSpeed = -20.0f;    // 目标转速 20 rad/s（轮表速度 10 m/s）
        //// 扭矩窗口：> 5.7 N·m 才爬得动 15° 坡（mg·sin15°·r），< 10.7 N·m 不抬轮
        //// （a < g·(轴距/2)/重心高 ≈ 4.77 m/s²）；150 N·m 会让小车起步即后空翻
        //def.maxMotorTorque = 5.0f;
        //axleL = static_cast<RevoluteJoint*>(world.createJoint(def));
        //def.initialize(plate, wheelR, Vector2(0.9f, 0.5f));
        //axleR = static_cast<RevoluteJoint*>(world.createJoint(def));
    //}

    //// --- 10 个金币触发器（车身高度，验证连续触发与无速度减损）---
    //for (int i = 0; i < 10; ++i) {
        //Box* coin = new Box(0.4f, 0.4f);
        //coin->isTrigger = true;
        //world.addBody(new Body(coin, 2.0f + i * 1.5f, 2.35f, 0.0f));
    //}

    //CoinListener listener;
    //world.setContactListener(&listener);

    //// --- 轨迹导出（输出到 2Ddemo 目录下）---
    //CSVExporter csv("2Ddemo/car_demo.csv");

    //// --- 模拟 10 秒 ---
    //float maxY = -1e9f;
    //int climbFrames = 0;          // 上坡期间持续前行的帧数（爬坡测试）
    //float minVxInCoinZone = 1e9f; // 金币+减速带区域内的最低车速（无减速减损测试）
    //for (int i = 0; i < 600; ++i) {
        //world.step(dt);
        //csv.writeFrame(i, world.getBodies());

        //float x = chassis->getPosition().getX();
        //float y = chassis->getPosition().getY();
        //float vx = chassis->getVelocity().getX();
        //maxY = std::max(maxY, y);
        //if (y > 2.8f && vx > 0.5f) climbFrames++;   // 高于起点且持续前进 = 正在爬坡
        //if (x > 2.0f && x < 15.5f) minVxInCoinZone = std::min(minVxInCoinZone, vx);

        //if ((i + 1) % 60 == 0) {
            //float springLenL = (springL->getAnchorB() - springL->getAnchorA()).length();
            //float springLenR = (springR->getAnchorB() - springR->getAnchorA()).length();
            //float axleErrL = (axleL->getAnchorB() - axleL->getAnchorA()).length();
            //Logger::info("t=" + std::to_string((i + 1) / 60) + "s" +
                //" x=" + std::to_string(x) + " y=" + std::to_string(y) +
                //" vx=" + std::to_string(vx) +
                //" wheelL=(" + std::to_string(wheelL->getPosition().getX()) + "," +
                //std::to_string(wheelL->getPosition().getY()) + ")" +
                //" wheelR=(" + std::to_string(wheelR->getPosition().getX()) + "," +
                //std::to_string(wheelR->getPosition().getY()) + ")" +
                //" torque=" + std::to_string(axleL->getMotorTorque(dt)) +
                //" axleErr=" + std::to_string(axleErrL) +
                //" coins=" + std::to_string(listener.triggerEnter));
        //}
    //}

    //// --- 验收断言 ---
    //bool coinOk = (listener.triggerEnter >= 10);                    // 10 枚金币全部触发过
    //// （悬挂振动会让车身在金币 AABB 边界进出多次，触发次数可能 > 10）
    //bool noSpeedLoss = (minVxInCoinZone > 0.5f);                    // 触发器无速度减损：全程持续前进
    //bool climbOk = (climbFrames >= 60) && (maxY > 6.0f);            // 15° 上坡稳定爬升
    //Logger::info("验收: coins=" + std::to_string(listener.triggerEnter) + "/10 -> " +
        //std::string(coinOk ? "PASS" : "FAIL"));
    //Logger::info("验收: 金币区最低车速=" + std::to_string(minVxInCoinZone) +
        //"（触发器不减速: " + std::string(noSpeedLoss ? "PASS" : "FAIL") + "）");
    //Logger::info("验收: 爬坡帧数=" + std::to_string(climbFrames) + " 最高y=" +
        //std::to_string(maxY) + " -> " + std::string(climbOk ? "PASS" : "FAIL"));
    //Logger::info("轨迹已导出: 2Ddemo/car_demo.csv");
    //Logger::info(std::string(">>> CarDemo ") + ((coinOk && noSpeedLoss && climbOk) ? "PASS" : "FAIL") + " <<<");
    //return (coinOk && noSpeedLoss && climbOk) ? 0 : 1;
//}
