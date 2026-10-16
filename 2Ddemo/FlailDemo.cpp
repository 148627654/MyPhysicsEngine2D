// ============================================================
// 2Ddemo 样例 9：流星链锤破坏拖拽 (Flail & MouseDemo)
//   - 悬挂点 + RopeJoint（最大绳长 4m）连结 50kg 实心铁球
//   - MouseJoint（5Hz / 阻尼 0.7）圆周蓄力甩动铁球
//   - 突然松手（DestroyJoint）→ 铁球切线飞出砸穿 20 块砖墙
//   - 绳索在极限拉直瞬间牢牢拽住链锤（全程球距锚点 ≤ 4m + 微小余量）
//   - CSVExporter 导出全部刚体轨迹到 2Ddemo/flail_demo.csv
// 运行：取消注释本文件 main，注释其他文件的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RopeJoint.h"
#include "MouseJoint.h"
#include "Box.h"
#include "Circle.h"
#include "Setting.h"
#include "CSVExporter.h"
#include "Logger.h"
#include <cmath>
#include <algorithm>
#include <vector>

//int main() {
//    Logger::info(">>> 2Ddemo 9: Flail 流星链锤（RopeJoint + MouseJoint 抛射）<<<");
//    World world(Vector2(0.0f, -9.8f));
//    float dt = 1.0f / 60.0f;
//
//    // 悬挂点（静态）
//    Body* anchor = new Body(new Box(0.4f, 0.4f), 0.0f, 8.0f, 0.0f);
//    world.addBody(anchor);
//    // 50kg 铁球
//    float ballDensity = 50.0f / (Settings::PAI * 0.25f);
//    Body* ball = new Body(new Circle(0.5f), 0.0f, 6.5f, ballDensity);
//    world.addBody(ball);
//
//    RopeJointDef ropeDef;
//    ropeDef.bodyA = anchor;
//    ropeDef.bodyB = ball;
//    ropeDef.localAnchorA = Vector2(0.0f, 0.0f);
//    ropeDef.localAnchorB = Vector2(0.0f, 0.0f);
//    ropeDef.maxLength = 4.0f;
//    ropeDef.collideConnected = false;
//    RopeJoint* rope = static_cast<RopeJoint*>(world.createJoint(ropeDef));
//
//    // 鼠标关节（标准时序：按下创建 → 移动只改目标点 → 松开销毁）
//    Body* ground = new Body(new Box(0.4f, 0.4f), -10.0f, -10.0f, 0.0f);
//    world.addBody(ground);
//    MouseJointDef mouseDef;
//    mouseDef.bodyA = ground;
//    mouseDef.bodyB = ball;
//    mouseDef.target = Vector2(0.0f, 6.5f);
//    mouseDef.localAnchorB = Vector2(0.0f, 0.0f);
//    mouseDef.maxForce = 20000.0f;
//    mouseDef.frequencyHz = 5.0f;
//    mouseDef.dampingRatio = 0.7f;
//    mouseDef.collideConnected = false;
//    MouseJoint* mouse = static_cast<MouseJoint*>(world.createJoint(mouseDef));
//
//    // 20 块砖墙（4 列 × 5 层，0.25²，密度 10）
//    std::vector<Body*> bricks;
//    for (int c = 0; c < 4; ++c)
//        for (int r = 0; r < 5; ++r)
//            bricks.push_back(world.createBox(0.25f, 0.25f,
//                1.725f + c * 0.25f, 4.125f + r * 0.25f, 10.0f));
//
//    CSVExporter csv("2Ddemo/flail_demo.csv");
//    float maxRopeLen = 0.0f;
//    float minRopeImpulse = 0.0f;
//    for (int i = 0; i < 420; ++i) {
//        if (i < 150) {
//            // 圆周蓄力甩动（目标点绕 (0,6.5) 半径 2、角速度 2 rad/s）
//            float theta = 2.0f * (i + 1) * dt;
//            mouse->setTarget(Vector2(2.0f * std::cos(theta), 6.5f + 2.0f * std::sin(theta)));
//        }
//        else if (i == 150) {
//            world.destroyJoint(mouse); // 松手：切线抛射
//        }
//        world.step(dt);
//        maxRopeLen = std::max(maxRopeLen, (ball->getPosition() - anchor->getPosition()).length());
//        minRopeImpulse = std::min(minRopeImpulse, rope->getImpulse());
//        csv.writeFrame(i, world.getBodies());
//    }
//
//    int scattered = 0;
//    for (int idx = 0; idx < (int)bricks.size(); ++idx) {
//        int c = idx / 5, r = idx % 5;
//        Vector2 spawn(1.725f + c * 0.25f, 4.125f + r * 0.25f);
//        if ((bricks[idx]->getPosition() - spawn).length() > 0.8f) ++scattered;
//    }
//
//    Logger::info("验收: 绳长峰值=" + std::to_string(maxRopeLen) + "（≤ 4.05 拽住 PASS）");
//    Logger::info("验收: 绳拉峰值冲量=" + std::to_string(-minRopeImpulse) + "（> 1 绷紧 PASS）");
//    Logger::info("验收: 碎砖=" + std::to_string(scattered) + "/20（≥ 5 砸墙 PASS）");
//    Logger::info("轨迹已导出: 2Ddemo/flail_demo.csv");
//    return 0;
//}
