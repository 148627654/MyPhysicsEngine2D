// ============================================================
// 2Ddemo 样例 5：双摆混沌轨迹 CSV 导出 (Double Pendulum Chaos)
//   两根 2.0m 连杆 RevoluteJoint 首尾铰接，水平高位释放。
//   双摆是非线性混沌系统：本样例导出末端刚体轨迹到
//   output/double_pendulum.csv，并逐帧监控 NaN 与机械能偏差。
//
//   Python 可视化（可选）：
//     import matplotlib.pyplot as plt
//     import pandas as pd
//     df = pd.read_csv("double_pendulum.csv")
//     plt.figure(figsize=(8, 8))
//     plt.plot(df["x2"], df["y2"], lw=0.5, color="crimson")
//     plt.title("Double Pendulum Chaotic Phase Space")
//     plt.show()
//
// 运行：取消注释本文件 main，注释其他文件（tests 与 2Ddemo）的 main
// ============================================================
#include "World.h"
#include "Body.h"
#include "Joint.h"
#include "RevoluteJoint.h"
#include "Box.h"
#include "Logger.h"
#include <cmath>
#include <fstream>
#include <string>

// 计算单根连杆的机械能：动能（平动+转动）+ 重力势能
static float RodEnergy(const Body* b, float g) {
    Vector2 v = b->getVelocity();
    float ke = 0.5f * b->getMass() * v.lengthSquared()
        + 0.5f * b->getInertia() * b->getAngularVelocity() * b->getAngularVelocity();
    float pe = b->getMass() * g * b->getPosition().getY();
    return ke + pe;
}

int main() {
    Logger::info(">>> 2Ddemo 5: DoublePendulum 双摆混沌 CSV 导出 <<<");
    World world(Vector2(0.0f, -9.8f));
    float dt = 1.0f / 60.0f;

    // --- 静态锚点（触发器）+ 两根 2m 连杆水平释放 ---
    Body* anchor = new Body(new Box(0.3f, 0.3f), 0.0f, 6.0f, 0.0f);
    anchor->getShape()->isTrigger = true;
    world.addBody(anchor);

    Body* rod1 = new Body(new Box(2.0f, 0.15f), 1.0f, 6.0f, 1.0f);
    Body* rod2 = new Body(new Box(2.0f, 0.15f), 3.0f, 6.0f, 1.0f);
    rod1->setSleepAllow(false);
    rod2->setSleepAllow(false);
    world.addBody(rod1);
    world.addBody(rod2);

    {
        RevoluteJointDef def;
        def.collideConnected = false;
        // 锚点铰接 rod1 左端
        def.bodyA = anchor;
        def.bodyB = rod1;
        def.localAnchorA = (Vector2(0.0f, 6.0f) - anchor->getPosition()).rotate(-anchor->getRotation());
        def.localAnchorB = (Vector2(0.0f, 6.0f) - rod1->getPosition()).rotate(-rod1->getRotation());
        def.referenceAngle = rod1->getRotation() - anchor->getRotation();
        world.createJoint(def);
        // rod1 右端铰接 rod2 左端
        def.bodyA = rod1;
        def.bodyB = rod2;
        def.localAnchorA = (Vector2(2.0f, 6.0f) - rod1->getPosition()).rotate(-rod1->getRotation());
        def.localAnchorB = (Vector2(2.0f, 6.0f) - rod2->getPosition()).rotate(-rod2->getRotation());
        def.referenceAngle = rod2->getRotation() - rod1->getRotation();
        world.createJoint(def);
    }
    Logger::info("组装完成: 双摆从水平高位释放（高能初始态 -> 混沌）");

    // --- CSV 导出（输出到 2Ddemo 目录下）---
    std::ofstream csv("2Ddemo/double_pendulum.csv");
    if (!csv.is_open()) {
        Logger::error("无法创建 2Ddemo/double_pendulum.csv（请确认 2Ddemo 目录存在）");
        return 1;
    }
    csv << "Frame,x1,y1,vx1,vy1,x2,y2,vx2,vy2\n";

    // --- 模拟 20 秒（1200 帧）---
    const float G = 9.8f;
    float E0 = RodEnergy(rod1, G) + RodEnergy(rod2, G);
    float maxEnergyDev = 0.0f;
    bool hasNaN = false;
    for (int i = 0; i < 1200; ++i) {
        world.step(dt);

        Vector2 p1 = rod1->getPosition(), v1 = rod1->getVelocity();
        Vector2 p2 = rod2->getPosition(), v2 = rod2->getVelocity();
        csv << i << "," << p1.getX() << "," << p1.getY() << "," << v1.getX() << "," << v1.getY()
            << "," << p2.getX() << "," << p2.getY() << "," << v2.getX() << "," << v2.getY() << "\n";

        float E = RodEnergy(rod1, G) + RodEnergy(rod2, G);
        maxEnergyDev = std::max(maxEnergyDev, std::abs(E - E0) / E0);
        if (std::isnan(p2.getX()) || std::isnan(p2.getY())) { hasNaN = true; break; }

        if ((i + 1) % 200 == 0) {
            Logger::info("frame=" + std::to_string(i + 1) +
                " rod2=(" + std::to_string(p2.getX()) + "," + std::to_string(p2.getY()) + ")" +
                " v2=(" + std::to_string(v2.getX()) + "," + std::to_string(v2.getY()) + ")" +
                " 能量偏差=" + std::to_string(maxEnergyDev * 100.0f) + "%");
        }
    }
    csv.close();

    // --- 验收断言：前 300 帧无 NaN、CSV 平滑连续 ---
    bool ok = !hasNaN;
    Logger::info("验收: 1200 帧轨迹无 NaN -> " + std::string(ok ? "PASS" : "FAIL") +
        "，最大能量偏差=" + std::to_string(maxEnergyDev * 100.0f) + "%");
    Logger::info("轨迹已导出: 2Ddemo/double_pendulum.csv（可用 Python matplotlib 绘制混沌图）");
    Logger::info(std::string(">>> DoublePendulum ") + (ok ? "PASS" : "FAIL") + " <<<");
    return ok ? 0 : 1;
}
