#pragma once
#include <algorithm>
#include <cmath>

namespace Physics2D {

    enum class CombineMode {
        Average,   // (A + B) / 2
        Minimum,   // min(A, B)
        Multiply,  // sqrt(A * B)
        Maximum    // max(A, B)
    };

    struct Material
    {
        float density = 1.0f;          // 密度
        float restitution = 0.0f;      // 恢复系数 [0, 1]
        float staticFriction = 0.5f;   // 静摩擦系数
        float dynamicFriction = 0.3f;  // 动摩擦系数

        CombineMode frictionCombine = CombineMode::Multiply;
        CombineMode restitutionCombine = CombineMode::Maximum;

        Material() = default;
		Material(float density, float restitution, float staticFriction, float dynamicFriction)
			: density(density), restitution(restitution), staticFriction(staticFriction),
			dynamicFriction(dynamicFriction){}

        static float Combine(float valA, float valB, CombineMode mode)
        {
			switch (mode) {
			case CombineMode::Average:
				return (valA + valB) / 2.0f;
			case CombineMode::Minimum:
				return std::min(valA, valB);
			case CombineMode::Multiply:
				return std::sqrt(valA * valB);
			case CombineMode::Maximum:
				return std::max(valA, valB);
			default:
				return 0.0f; // Default case, should not happen
			}
        }
    };
}