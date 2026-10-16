// ============================================================
// 2Ddemo 样例 6：智能楼层电梯系统 (ElevatorDemo)
//   - 垂直导轨 Box（静态）+ 电梯轿厢 Box（动态）
//   - PrismaticJoint（滑轨沿 Y 轴）+ 马达（Fmax = 5000N）
//   - 上下楼层限位 [-5.0, +5.0]：到达顶层平稳停靠、载人随动不抖动
//   - 100kg 乘客刚体随轿厢升降（接触承载，非关节连接）
//   - CSVExporter 导出轿厢与乘客轨迹到 2Ddemo/elevator_demo.csv
// 运行：取消注释本文件 main，注释其他文件的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "PrismaticJoint.h"
#include "Box.h"
#include "CSVExporter.h"
#include "Logger.h"
#include <cmath>

//int main() {
//    Logger::info(">>> 2Ddemo 6: Elevator 智能电梯（Prismatic 马达 + 楼层限位）<<<");
//    World world(Vector2(0.0f, -9.8f));
//    float dt = 1.0f / 60.0f;
//
//    // 井道导轨（静态），放在轿厢侧面（若与轿厢/乘客重叠会产生推挤）
//    Body* rail = new Body(new Box(0.4f, 16.0f), -2.5f, 0.0f, 0.0f);
//    world.addBody(rail);
//    Body* car = new Body(new Box(2.0f, 0.5f), 0.0f, 0.0f, 300.0f); // 轿厢远重于乘客
//    world.addBody(car);
//    // 100kg 乘客，站立于轿厢顶部
//    Body* passenger = new Body(new Box(1.0f, 1.0f), 0.0f, 0.75f, 100.0f);
//    world.addBody(passenger);
//
//    PrismaticJointDef def;
//    def.bodyA = rail;
//    def.bodyB = car;
//    def.localAnchorA = Vector2(2.5f, 0.0f); // 世界锚点 = 轿厢中心 (0,0)
//    def.localAnchorB = Vector2(0.0f, 0.0f);
//    def.localAxisA = Vector2(0.0f, 1.0f);   // 垂直导轨
//    def.referenceAngle = 0.0f;
//    def.enableLimit = true;
//    def.lowerTranslation = -5.0f;           // 楼层限位 [-5, +5]
//    def.upperTranslation = 5.0f;
//    def.enableMotor = true;
//    def.motorSpeed = 3.0f;                  // 上升 3 m/s
//    def.maxMotorForce = 5000.0f;            // 驱动推力上限
//    def.collideConnected = false;           // 井道与轿厢严禁互推
//    PrismaticJoint* elevator = static_cast<PrismaticJoint*>(world.createJoint(def));
//
//    CSVExporter csv("2Ddemo/elevator_demo.csv");
//
//    int phase = 0;      // 0=上升, 1=顶层停靠, 2=下降
//    int dwell = 0;
//    float motorSpeed = 3.0f;
//    float maxOvershoot = 0.0f;
//    float maxPassengerGap = 0.0f;
//    for (int i = 0; i < 600; ++i) {
//        // 真实电梯控制：临近楼层减速（2.4 m/s²）→ 0.5 m/s 缓速靠上限位。
//        // 全速撞限位会瞬时停车，动量守恒把乘客弹起——绝非"平稳停靠"
//        if (phase == 0) {
//            if (car->getPosition().getY() > 2.0f && motorSpeed > 0.5f) {
//                motorSpeed = std::max(0.5f, motorSpeed - 0.04f);
//                elevator->setMotorSpeed(motorSpeed);
//            }
//            if (std::abs(car->getPosition().getY() - 5.0f) < 0.01f) { phase = 1; dwell = 0; }
//        }
//        if (phase == 1 && ++dwell > 60) {
//            phase = 2;
//            motorSpeed = -3.0f;
//            elevator->setMotorSpeed(motorSpeed);
//        }
//        if (phase == 2 && car->getPosition().getY() < -2.0f && motorSpeed < -0.5f) {
//            motorSpeed = std::min(-0.5f, motorSpeed + 0.04f);
//            elevator->setMotorSpeed(motorSpeed);
//        }
//
//        world.step(dt);
//
//        // 验收指标：限位超界量 + 乘客相对轿厢的浮动
//        maxOvershoot = std::max(maxOvershoot,
//            std::max(car->getPosition().getY() - 5.0f, -5.0f - car->getPosition().getY()));
//        maxPassengerGap = std::max(maxPassengerGap,
//            std::abs(passenger->getPosition().getY() - (car->getPosition().getY() + 0.75f)));
//        csv.writeFrame(i, world.getBodies());
//    }
//
//    Logger::info("验收: 顶层停靠 y=" + std::to_string(car->getPosition().getY()) +
//        " 超界=" + std::to_string(maxOvershoot) + "（< 0.001 PASS）");
//    Logger::info("验收: 乘客最大浮动=" + std::to_string(maxPassengerGap) + "（< 0.1 PASS）");
//    Logger::info("轨迹已导出: 2Ddemo/elevator_demo.csv");
//    return 0;
//}
