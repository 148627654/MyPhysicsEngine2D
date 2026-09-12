#pragma once
#include "../Collision/Manifold.h"

// 只写声明，告诉编译器有这个函数
void ImpulseSolver(Manifold& m);
void PositionalCorrection(Manifold& m);