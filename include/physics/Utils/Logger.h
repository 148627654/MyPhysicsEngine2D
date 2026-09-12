#pragma once
#include <iostream>
#include <string>
#include "../Collision/Manifold.h"

class Logger {
public:
	// ANSI 颜色代码
	enum Color {
		RESET = 0 ,
		GREEN = 32 ,
		YELLOW = 33 ,
		RED = 31 ,
		CYAN = 36 ,
		WHITE = 37
	};

	static void Info(const std::string& message);
	static void Warning(const std::string& message);
	static void Error(const std::string& message);

	// 专门用于记录碰撞流形的函数
	static void LogCollision(const Manifold& m);

private:
	static void SetColor(Color color);
};