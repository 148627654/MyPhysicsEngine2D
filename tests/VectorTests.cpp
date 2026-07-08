#include <iostream>
#include <cassert>
#include <iomanip>
#include "../include/physics/Common/Vector2.h" // 根据你的路径修改

void TestVectorMath() {
    std::cout << "Running Vector2 Tests...\n";

    // 1. 基础加减
    Vector2 v1(3.0f, 4.0f);
    Vector2 v2(1.0f, 2.0f);
    assert((v1 + v2) == Vector2(4.0f, 6.0f));
    assert((v1 - v2) == Vector2(2.0f, 2.0f));
    std::cout << "  [OK] Addition and Subtraction\n";

    // 2. 标量乘除 (测试此项前建议先修改参数类型为 float 而非 float&)
    float scale = 2.0f;
    assert((v1 * scale) == Vector2(6.0f, 8.0f));
    assert((v1 / scale) == Vector2(1.5f, 2.0f));
    std::cout << "  [OK] Scalar Multiplication and Division\n";

    // 3. 长度计算
    assert(v1.Length() == 5.0f);
    assert(v1.LengthSquared() == 25.0f);
    std::cout << "  [OK] Length Calculations\n";

    // 4. 归一化与安全检查
    Vector2 vZero(0, 0);
    assert(vZero.Normalize() == Vector2(0, 0)); // 检查防崩溃逻辑
    Vector2 vNorm = v1.Normalize();
    assert(std::abs(vNorm.Length() - 1.0f) < 0.001f);
    std::cout << "  [OK] Normalization and Safety\n";

    // 5. 点积
    assert(v1.Dot(v2) == (3.0f * 1.0f + 4.0f * 2.0f));
    std::cout << "  [OK] Dot Product\n";

    // 6. 旋转 (90度 = PI/2)
    Vector2 right(1.0f, 0.0f);
    Vector2 up = right.Rotate(1.570796f); // 约等于 PI/2
    assert(std::abs(up.getX() - 0.0f) < 0.001f);
    assert(std::abs(up.getY() - 1.0f) < 0.001f);
    std::cout << "  [OK] Rotation\n";

    // 7. 垂直向量
    Vector2 v3(1.0f, 0.0f);
    assert(v3.GetLeftNormal() == Vector2(0.0f, 1.0f));
    assert(v3.GetRightNormal() == Vector2(0.0f, -1.0f));
    std::cout << "  [OK] Normals\n";

    std::cout << "All Vector2 Tests Passed Successfully!\n\n";
}

//int main() {
//    try {
//        TestVectorMath();
//    }
//    catch (const std::exception& e) {
//        std::cerr << "Test failed with error: " << e.what() << std::endl;
//        return -1;
//    }
//    return 0;
//}