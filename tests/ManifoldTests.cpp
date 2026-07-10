#include <iostream>
#include <iomanip>
#include <vector>
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Collision/Circle.h"
#include "../include/physics/Collision/Collision.h"
#include "../include/physics/Collision/Manifold.h"



// 辅助函数：打印流形数据
void PrintResult(const std::string& testName, bool collided, const Manifold& m) {
    std::cout << "==== Test: " << testName << " ====" << std::endl;
    if (!collided) {
        std::cout << "Result: No Collision" << std::endl;
    }
    else {
        std::cout << "Result: COLLISION!" << std::endl;
        std::cout << "  Penetration: " << m.penetration << std::endl;
        std::cout << "  Normal:      (" << m.normal.getX() << ", " << m.normal.getY() << ")" << std::endl;
        if (!m.contacts.empty()) {
            std::cout << "  Contact Pt:  (" << m.contacts[0].getX() << ", " << m.contacts[0].getY() << ")" << std::endl;
        }
    }
    std::cout << "--------------------------------" << std::endl << std::endl;
}

int main() {
    // 1. 初始化两个圆的形状
    Circle *circle1=new Circle(1.0f); // 半径 1.0
    Circle *circle2=new Circle(1.0f); // 半径 1.0

    // 2. 创建两个刚体
    Body bodyA(circle1);
    bodyA.SetShape(circle1, 1.0f);
    Body bodyB(circle2); 
    bodyB.SetShape(circle2, 1.0f);

    // --- 情况 1: 完全不接触 ---
    bodyA.SetPosition(0.0f, 0.0f);
    bodyB.SetPosition(5.0f, 0.0f); // 距离 5，半径和 2
    Manifold m1(&bodyA, &bodyB);
    bool res1 = Collision::CircleVsCircle(&m1, &bodyA, &bodyB);
    PrintResult("Far Away (No Collision)", res1, m1);

    // --- 情况 2: 发生重叠 (水平) ---
    bodyA.SetPosition(0.0f, 0.0f);
    bodyB.SetPosition(1.5f, 0.0f); // 距离 1.5，半径和 2，穿透 0.5
    Manifold m2(&bodyA, &bodyB);
    bool res2 = Collision::CircleVsCircle(&m2, &bodyA, &bodyB);
    PrintResult("Overlapping (Horizontal)", res2, m2);

    // --- 情况 3: 刚好接触 (切点) ---
    bodyA.SetPosition(0.0f, 0.0f);
    bodyB.SetPosition(0.0f, 2.0f); // 距离 2.0，半径和 2.0
    Manifold m3(&bodyA, &bodyB);
    bool res3 = Collision::CircleVsCircle(&m3, &bodyA, &bodyB);
    PrintResult("Just Touching (Edge Case)", res3, m3);

    // --- 情况 4: 圆心完全重合 ---
    bodyA.SetPosition(10.0f, 10.0f);
    bodyB.SetPosition(10.0f, 10.0f); // 距离 0
    Manifold m4(&bodyA, &bodyB);
    bool res4 = Collision::CircleVsCircle(&m4, &bodyA, &bodyB);
    PrintResult("Identical Positions (Concentric)", res4, m4);

    return 0;
}