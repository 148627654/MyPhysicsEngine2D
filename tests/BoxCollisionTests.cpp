#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Collision/Collision.h"
#include "../include/physics/Collision/Manifold.h"

// 打印测试结果的辅助函数
void PrintBoxResult(const std::string& testName, bool collided, const Manifold& m) {
    std::cout << "==== Test: " << testName << " ====" << std::endl;
    if (!collided) {
        std::cout << "Result: [ NO COLLISION ]" << std::endl;
    }
    else {
        std::cout << "Result: [ COLLISION! ]" << std::endl;
        std::cout << "  Penetration: " << std::fixed << std::setprecision(3) << m.penetration << std::endl;
        std::cout << "  Normal:      (" << m.normal.getX() << ", " << m.normal.getY() << ")" << std::endl;
        // 注意：目前你可能还没写复杂的接触点裁剪，这里先观察法线和深度
    }
    std::cout << "--------------------------------" << std::endl << std::endl;
}

int main() {
    // 1. 创建形状：两个 2x2 的正方形
    Box* boxA_shape = new Box(2.0f, 2.0f);
    Box* boxB_shape = new Box(2.0f, 2.0f);

    // 2. 创建刚体 (使用你 Body.h 的构造函数)
    // Body(Shape* s, float x=0.0, float y=0.0, float density = 1.0f)
    Body bodyA(boxA_shape, 0.0f, 0.0f, 1.0f);
    Body bodyB(boxB_shape, 0.0f, 0.0f, 1.0f);

    // --- 情况 1: 轴对齐且完全不接触 ---
    bodyA.SetPosition(0.0f, 0.0f);
    bodyB.SetPosition(5.0f, 0.0f);
    Manifold m1(&bodyA, &bodyB);
    bool res1 = Collision::BoxVsBox(&m1, &bodyA, &bodyB);
    PrintBoxResult("Far Away (AABB)", res1, m1);

    // --- 情况 2: 轴对齐重叠 (水平方向) ---
    // A(0,0), B(1.5, 0)。 A的右边缘在 1.0, B的左边缘在 0.5。重叠应为 0.5
    bodyA.SetPosition(0.0f, 0.0f);
    bodyB.SetPosition(1.5f, 0.0f);
    Manifold m2(&bodyA, &bodyB);
    bool res2 = Collision::BoxVsBox(&m2, &bodyA, &bodyB);
    PrintBoxResult("Horizontal Overlap (AABB)", res2, m2);

    // --- 情况 3: 旋转碰撞 (SAT 的核心价值) ---
    // A 固定在原点。B 在 (2, 2) 且旋转 45 度 (约 0.785 弧度)
    // 此时 B 的顶点会伸向 A，判定是否发生斜向碰撞
    bodyA.SetPosition(0.0f, 0.0f);
    bodyB.SetPosition(2.0f, 2.0f);
    // 修改 Body 增加 SetRotation 或直接通过实例设置
    // 假设你已经在 Body 类中实现了设置旋转的方法
    // bodyB.rotation = 0.78539f; // 45 degrees

    // 如果你还没有 SetRotation 方法，可以临时给 Body 加一个，或者直接修改成员
    // 为了测试，这里建议手动设置一下旋转
    Manifold m3(&bodyA, &bodyB);
    bool res3 = Collision::BoxVsBox(&m3, &bodyA, &bodyB);
    PrintBoxResult("Diagonal (Rotated) - No Rotation yet", res3, m3);

    // --- 情况 4: 包含情况 (一个小矩形在大的内部) ---
    Box* bigBox = new Box(10.0f, 10.0f);
    Box* smallBox = new Box(2.0f, 2.0f);
    Body bodyBig(bigBox, 0.0f, 0.0f, 1.0f);
    Body bodySmall(smallBox, 0.1f, 0.1f, 1.0f);
    Manifold m4(&bodyBig, &bodySmall);
    bool res4 = Collision::BoxVsBox(&m4, &bodyBig, &bodySmall);
    PrintBoxResult("Containment (Small inside Big)", res4, m4);

    return 0;
}