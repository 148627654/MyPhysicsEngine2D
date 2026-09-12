#include <iostream>
#include <iomanip>
#include <vector>
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Collision/Circle.h"
#include "../include/physics/Collision/Box.h"
#include "../include/physics/Collision/Collision.h"
#include "../include/physics/Collision/Manifold.h"

// 辅助打印函数
//void PrintMixedResult(const std::string& testName, bool collided, const Manifold& m) {
//    std::cout << "==== Test: " << testName << " ====" << std::endl;
//    if (!collided) {
//        std::cout << "Result: [ NO COLLISION ]" << std::endl;
//    }
//    else {
//        std::cout << "Result: [ COLLISION! ]" << std::endl;
//        std::cout << "  Penetration: " << std::fixed << std::setprecision(3) << m.penetration << std::endl;
//        std::cout << "  Normal:      (" << m.normal.getX() << ", " << m.normal.getY() << ")" << std::endl;
//        if (!m.contacts.empty()) {
//            std::cout << "  Contact Pt:  (" << m.contacts[0].getX() << ", " << m.contacts[0].getY() << ")" << std::endl;
//        }
//    }
//    std::cout << "--------------------------------" << std::endl << std::endl;
//}

//int main() {
//    // 1. 初始化形状
//    Circle* circleShape = new Circle(1.0f); // 半径 1.0
//    Box* boxShape = new Box(2.0f, 2.0f);    // 2x2 矩形 (hw=1, hh=1)
//
//    // 2. 创建刚体 (A为圆，B为矩形)
//    Body bodyA(circleShape, 0.0f, 0.0f, 1.0f);
//    Body bodyB(boxShape, 0.0f, 0.0f, 1.0f);
//
//    // --- Case 1: 轴对齐 - 从右侧水平撞击 ---
//    // Box在(0,0)，圆在(1.8, 0)。圆左边缘在0.8，Box右边缘在1.0。
//    // 预期：撞击，穿透深度 0.2，法线从圆指向Box (1, 0)
//    bodyA.SetPosition(1.8f, 0.0f);
//    bodyB.SetPosition(0.0f, 0.0f);
//    Manifold m1(&bodyA, &bodyB);
//    bool res1 = Collision::CircleVsBox(&m1, &bodyA, &bodyB);
//    PrintMixedResult("Side Hit (Horizontal)", res1, m1);
//
//    // --- Case 2: 撞击旋转后的矩形顶点 ---
//    // Box在(0,0)，旋转45度。此时顶点在 (1.414, 0) 附近。
//    // 圆在 (2.0, 0)，半径1.0。圆左边缘在1.0。
//    // 预期：撞击顶点。
//    bodyA.SetPosition(2.0f, 0.0f);
//    bodyB.SetPosition(0.0f, 0.0f);
//    bodyB.SetRotation(0.78539f); // 45 度
//    Manifold m2(&bodyA, &bodyB);
//    bool res2 = Collision::CircleVsBox(&m2, &bodyA, &bodyB);
//    PrintMixedResult("Rotated Corner Hit", res2, m2);
//
//    // --- Case 3: 圆心在矩形内部 (Deep Penetration) ---
//    // Box在(0,0)，圆在(0.5, 0)。圆心在Box内。
//    // 预期：触发 Inside 逻辑，法线指向最近的右边 (1, 0)，穿透深度为 1.0(半径) + 0.5(到右边距离) = 1.5
//    bodyA.SetPosition(0.5f, 0.0f);
//    bodyB.SetPosition(0.0f, 0.0f);
//    bodyB.SetRotation(0.0f); // 重置旋转
//    Manifold m3(&bodyA, &bodyB);
//    bool res3 = Collision::CircleVsBox(&m3, &bodyA, &bodyB);
//    PrintMixedResult("Center Inside Box", res3, m3);
//
//    // --- Case 4: 刚好不接触 (Corner Case) ---
//    // 圆在 (2.1, 2.1)，Box在 (0,0)。距离顶点 (1,1) 的距离约为 1.55，大于半径 1.0
//    // 预期：无碰撞
//    bodyA.SetPosition(2.1f, 2.1f);
//    bodyB.SetPosition(0.0f, 0.0f);
//    Manifold m4(&bodyA, &bodyB);
//    bool res4 = Collision::CircleVsBox(&m4, &bodyA, &bodyB);
//    PrintMixedResult("Just Missed (Diagonal)", res4, m4);
//
//    return 0;
//}