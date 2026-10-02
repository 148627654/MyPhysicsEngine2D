#pragma once
#include "../Collision/Manifold.h"

// 只写声明，告诉编译器有这个函数
void impulseSolver(Manifold& m);
void positionalCorrection(Manifold& m);