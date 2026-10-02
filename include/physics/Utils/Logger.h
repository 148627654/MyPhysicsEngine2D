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

	static void info(const std::string& message);
	static void warning(const std::string& message);
	static void error(const std::string& message);

	// 专门用于记录碰撞流形的函数
	static void logCollision(const Manifold& m);

private:
	static void setColor(Color color);
};