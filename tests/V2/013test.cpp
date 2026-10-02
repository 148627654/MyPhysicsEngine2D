//#include "../Dynamics/World.h"
//#include "../Dynamics/Body.h"
//#include "../Collision/Box.h"
//#include "../Utils/Logger.h"
//#include "../Utils/CSVExporter.h"
//
//void TestHighSpeedInterception() {
//    Logger::info(">>> Starting Day 13 Test: Ultimate Bullet Interception <<<");
//    CSVExporter exporter("output/v2_013.csv");
//
//    // 1. 初始化 (关闭重力，纯水平拦截测试)
//    World world(Vector2(0, 0));
//    float dt = 1.0f / 60.0f;
//
//    // 2. 创建一堵极薄的墙 (Static, ID: 0)
//    // 放在 X=0 处，厚度只有 0.1m
//    Body* wall = new Body(new Box(0.1f, 10.0f), 0.0f, 0.0f, 0.0f);
//    world.addBody(wall);
//    Logger::info("Thin wall created at X=0, Thickness=0.1");
//
//    // 3. 创建高速子弹 (Dynamic, ID: 1)
//    // 初始位置在 -5.0，速度 300m/s，每帧移动 5.0m！
//    // 如果没有 CCD，第一帧在 -5.0，第二帧就直接跳到了 0.0 墙后！
//    Body* bullet = new Body(new Box(0.5f, 0.5f), -5.0f, 0.0f, 1.0f);
//    bullet->setBullet(true); // 【开启 CCD】
//    bullet->setAwake(true);
//    bullet->setRestitution(0.5f); // 设置反弹系数，验证子步冲量解算
//    bullet->setVelocity(Vector2(300.0f, 0.0f));
//    world.addBody(bullet);
//
//    Logger::info("Bullet spawned at X=-5.0, Velocity=300m/s (5.0m per frame)");
//
//    bool intercepted = false;
//
//    // 4. 模拟运行
//    for (int frame = 0; frame < 20; ++frame) {
//        // 在步进前记录当前位置
//        exporter.writeFrame(frame, world.getBodies());
//
//        // 执行物理步进 (内部包含位置预测、TOI回溯、冲量解算)
//        world.step(dt);
//
//        float bulletX = bullet->getPosition().getX();
//        float bulletVX = bullet->getVelocity().getX();
//
//        // 验证拦截：子弹位置不能 > 0
//        if (!intercepted && bulletX >= -0.26f) { // 考虑到子弹半径 0.25 和 SLOP
//            Logger::info("--- Analysis at Frame " + std::to_string(frame) + " ---");
//            Logger::info("Bullet PosX: " + std::to_string(bulletX));
//            Logger::info("Bullet VelX: " + std::to_string(bulletVX));
//
//            if (bulletVX < 0) {
//                Logger::info("SUCCESS: Velocity reversed! Bullet was blocked and bounced.");
//                intercepted = true;
//            }
//        }
//    }
//
//    
//
//    Logger::info("Data exported to output/v2_013_interception.csv");
//}
//
//int main() {
//    TestHighSpeedInterception();
//    return 0;
//}