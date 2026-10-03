// ============================================================
// 2Ddemo 样例 4：闭环四连杆大考 (Closed-Loop 4-Bar Linkage)
//   4 根连杆 Box + 4 个 RevoluteJoint 闭合成平行四边形回路。
//   树状结构（单摆/链条）容易解算，但关节首尾相接形成闭环（Loop Closure）
//   极易造成过约束方程死锁 —— 本样例在重力下自由翻滚 1000 帧（16.7 秒），
//   监控 4 个闭环销钉的锚点位移误差，验证 Sequential Impulses 解算器的
//   绝对稳定性：误差全程 < 1e-3 米、无锁死爆炸。
// 运行：取消注释本文件 main，注释其他文件（tests 与 2Ddemo）的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RevoluteJoint.h"
#include "Box.h"
#include "Logger.h"
#include <cmath>
#include <vector>

//int main() {
    //Logger::info(">>> 2Ddemo 4: ClosedLoop 闭环四连杆（1000 帧稳定性大考）<<<");
    //World world(Vector2(0.0f, -9.8f));
    //float dt = 1.0f / 60.0f;

    //// --- 平行四边形四连杆（边长 2，倾角 0.4 rad）---
    //const float theta = 0.4f;
    //const Vector2 d(2.0f * std::cos(theta), 2.0f * std::sin(theta)); // 对边偏移
    //Vector2 p0(-1.0f, 4.0f), p1(1.0f, 4.0f);          // 底边两端
    //Vector2 p2 = p1 + d, p3 = p0 + d;                  // 顶边两端

    //Body* rod1 = new Body(new Box(2.0f, 0.25f), ((p0 + p1) * 0.5f).getX(), ((p0 + p1) * 0.5f).getY(), 1.0f);
    //Body* rod2 = new Body(new Box(2.0f, 0.25f), ((p1 + p2) * 0.5f).getX(), ((p1 + p2) * 0.5f).getY(), 1.0f);
    //Body* rod3 = new Body(new Box(2.0f, 0.25f), ((p2 + p3) * 0.5f).getX(), ((p2 + p3) * 0.5f).getY(), 1.0f);
    //Body* rod4 = new Body(new Box(2.0f, 0.25f), ((p3 + p0) * 0.5f).getX(), ((p3 + p0) * 0.5f).getY(), 1.0f);
    //rod1->setRotation(0.0f);
    //rod2->setRotation(theta);
    //rod3->setRotation(0.0f);
    //rod4->setRotation(theta + Settings::PAI);
    //for (Body* r : { rod1, rod2, rod3, rod4 }) {
        //r->setSleepAllow(false);
        //world.addBody(r);
    //}

    //// --- 4 个闭环销钉（collideConnected=false：杆端本就重叠装配）---
    //std::vector<RevoluteJoint*> pins;
    //{
        //RevoluteJointDef def;
        //def.collideConnected = false;
        //def.initialize(rod1, rod2, p1); pins.push_back(static_cast<RevoluteJoint*>(world.createJoint(def)));
        //def.initialize(rod2, rod3, p2); pins.push_back(static_cast<RevoluteJoint*>(world.createJoint(def)));
        //def.initialize(rod3, rod4, p3); pins.push_back(static_cast<RevoluteJoint*>(world.createJoint(def)));
        //def.initialize(rod4, rod1, p0); pins.push_back(static_cast<RevoluteJoint*>(world.createJoint(def)));
    //}
    //Logger::info("组装完成: 4 连杆 + 4 闭环销钉（平行四边形，自由翻滚）");

    //// 初始微扰：破坏静力平衡，让机构真正翻滚起来
    //rod1->setVelocity(Vector2(0.5f, 0.0f));
    //rod1->setAngularVelocity(0.3f);

    //// --- 1000 帧稳定性大考 ---
    //float maxClosureError = 0.0f;
    //bool hasNaN = false;
    //for (int i = 0; i < 1000; ++i) {
        //world.step(dt);

        //float frameErr = 0.0f;
        //for (RevoluteJoint* j : pins) {
            //float err = (j->getAnchorB() - j->getAnchorA()).length();
            //frameErr = std::max(frameErr, err);
        //}
        //maxClosureError = std::max(maxClosureError, frameErr);
        //if (std::isnan(rod1->getPosition().getX()) || std::isnan(rod2->getPosition().getY())) {
            //hasNaN = true;
            //break;
        //}

        //if ((i + 1) % 100 == 0) {
            //Logger::info("frame=" + std::to_string(i + 1) +
                //" 闭环误差=" + std::to_string(frameErr) +
                //" rod1=(" + std::to_string(rod1->getPosition().getX()) + "," +
                //std::to_string(rod1->getPosition().getY()) + ")" +
                //" rod2rot=" + std::to_string(rod2->getRotation()));
        //}
    //}

    //// --- 验收断言：1000 帧闭环点漂移 < 1e-3 米 ---
    //bool ok = !hasNaN && (maxClosureError < 1e-3f);
    //Logger::info("验收: 1000 帧闭环最大漂移=" + std::to_string(maxClosureError) +
        //"（阈值 1e-3）NaN=" + std::string(hasNaN ? "有" : "无") +
        //" -> " + std::string(ok ? "PASS" : "FAIL"));
    //Logger::info(std::string(">>> ClosedLoopLinkage ") + (ok ? "PASS" : "FAIL") + " <<<");
    //return ok ? 0 : 1;
//}
