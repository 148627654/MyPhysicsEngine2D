//#include "../Dynamics/World.h"
//#include "../Dynamics/Body.h"
//#include "../Collision/Box.h"
//#include "../Collision/Circle.h"
//#include "../Utils/Logger.h"
//#include "../Utils/CSVExporter.h"
//
//void RunSingleTOITest(Shape* targetShape, Shape* bulletShape, std::string filename, std::string label) {
//    Logger::Info(">>> Starting TOI Test: " + label);
//    CSVExporter exporter(filename);
//
//    // 初始化：关闭重力，纯线性测试
//    World world(Vector2(0, 0));
//    float dt = 1.0f / 60.0f;
//
//    // 1. 创建静态目标 (在原点 X=0)
//    Body* target = new Body(targetShape, 0.0f, 0.0f, 0.0f);
//    world.AddBody(target);
//
//    // 2. 创建高速子弹 (从 X=-5.0 冲向 X=0)
//    // 速度 120m/s，每帧移动 2.0m。第3帧会跳过中心。
//    Body* bullet = new Body(bulletShape, -5.0f, 0.0f, 1.0f);
//    bullet->SetBullet(true);
//    bullet->setAwake(true);
//    bullet->SetVelocity(Vector2(120.0f, 0.0f));
//    world.AddBody(bullet);
//
//    // 3. 模拟 10 帧
//    for (int frame = 0; frame < 10; ++frame) {
//        world.Step(dt);
//        exporter.WriteFrame(frame, world.GetBodies());
//
//        // 检查碰撞对中的 TOI
//        auto const& contactMap = world.getContactMap();
//        for (auto const& [key, contact] : contactMap) {
//            if (contact->m_toi < 1.0f) {
//                Logger::Info("[" + label + "] COLLISION DETECTED!");
//                Logger::Info("   Frame: " + std::to_string(frame));
//                Logger::Info("   Alpha (Time of Impact): " + std::to_string(contact->m_toi));
//                Logger::Info("   Bullet X: " + std::to_string(bullet->GetPosition().getX()));
//            }
//        }
//    }
//    Logger::Info("Test " + label + " finished. Data: " + filename + "\n");
//}
//
//void RunDay12Tests() {
//    // A. Circle vs Circle
//    RunSingleTOITest(new Circle(1.0f), new Circle(0.5f),
//        "output/v2_012_circle_circle.csv", "Circle-Circle");
//
//    // B. Box vs Circle
//    RunSingleTOITest(new Box(2.0f, 10.0f), new Circle(0.5f),
//        "output/v2_012_box_circle.csv", "Box-Circle");
//
//    // C. Box vs Box
//    RunSingleTOITest(new Box(2.0f, 10.0f), new Box(1.0f, 1.0f),
//        "output/v2_012_box_box.csv", "Box-Box");
//}
//
//int main() {
//    RunDay12Tests();
//    return 0;
//}