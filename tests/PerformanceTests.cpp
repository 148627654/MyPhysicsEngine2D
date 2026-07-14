#include <iostream>
#include <vector>
#include <chrono> // 用于高精度计时
#include <random> // 用于生成随机位置
#include "../include/physics/Dynamics/Body.h"
#include "../include/physics/Collision/Circle.h"
#include "../include/physics/Collision/Collision.h"
#include "../include/physics/Collision/Manifold.h"

// 模拟场景设置
const int BODY_COUNT = 500;        // 物体数量（越多越能看出差距）
const float WORLD_SIZE = 1000.0f;  // 世界范围 1000x1000

//int main() {
//    std::cout << "==== Day 07: Broadphase (AABB) Performance Test ====" << std::endl;
//    std::cout << "Testing with " << BODY_COUNT << " bodies..." << std::endl << std::endl;
//
//    // 1. 初始化物体
//    std::vector<Body*> bodies;
//    Circle* circleShape = new Circle(1.0f); // 统一使用半径为 1 的圆
//
//    std::mt19937 rng(42); // 固定随机种子保证测试可复现
//    std::uniform_real_distribution<float> dist(0, WORLD_SIZE);
//
//    for (int i = 0; i < BODY_COUNT; ++i) {
//        Body* b = new Body(circleShape, dist(rng), dist(rng), 1.0f);
//        b->SetRotation(dist(rng)); // 随机旋转
//        b->updateAABB();           // 手动更新一次初始 AABB
//        bodies.push_back(b);
//    }
//
//    // --- 测试 1: 纯窄相检测 (无 AABB 过滤) ---
//    auto start1 = std::chrono::high_resolution_clock::now();
//    int narrowCount1 = 0;
//    int collisionCount1 = 0;
//
//    for (int i = 0; i < BODY_COUNT; ++i) {
//        for (int j = i + 1; j < BODY_COUNT; ++j) {
//            narrowCount1++; // 计数：进入复杂计算的次数
//            Manifold m(bodies[i], bodies[j]);
//            if (Collision::Dispatch(&m, bodies[i], bodies[j])) {
//                collisionCount1++;
//            }
//        }
//    }
//    auto end1 = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double, std::milli> time1 = end1 - start1;
//
//    // --- 测试 2: 开启 AABB 过滤 (Broadphase) ---
//    auto start2 = std::chrono::high_resolution_clock::now();
//    int narrowCount2 = 0;
//    int collisionCount2 = 0;
//
//    for (int i = 0; i < BODY_COUNT; ++i) {
//        for (int j = i + 1; j < BODY_COUNT; ++j) {
//            // 先进行极快的 AABB 判断
//            if (Collision::AABBvsAABB(bodies[i]->GetAABB(), bodies[j]->GetAABB())) {
//                narrowCount2++; // 只有 AABB 撞了才进入窄相
//                Manifold m(bodies[i], bodies[j]);
//                if (Collision::Dispatch(&m, bodies[i], bodies[j])) {
//                    collisionCount2++;
//                }
//            }
//        }
//    }
//    auto end2 = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double, std::milli> time2 = end2 - start2;
//
//    // --- 输出结果 ---
//    std::cout << "[Test 1: No Filtering]" << std::endl;
//    std::cout << "  Narrowphase checks: " << narrowCount1 << std::endl;
//    std::cout << "  Execution Time:     " << time1.count() << " ms" << std::endl << std::endl;
//
//    std::cout << "[Test 2: With AABB Filtering]" << std::endl;
//    std::cout << "  Narrowphase checks: " << narrowCount2 << std::endl;
//    std::cout << "  Skipped checks:      " << (narrowCount1 - narrowCount2) << " ("
//        << (float)(narrowCount1 - narrowCount2) / narrowCount1 * 100.0f << "%)" << std::endl;
//    std::cout << "  Execution Time:     " << time2.count() << " ms" << std::endl << std::endl;
//
//    std::cout << "Performance Boost: " << (time1.count() / time2.count()) << "x faster!" << std::endl;
//
//    // 清理内存
//    for (auto b : bodies) delete b;
//    delete circleShape;
//
//    return 0;
//}