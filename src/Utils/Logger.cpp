#include "Logger.h"

void Logger::SetColor(Color color) {
    // \033[1;...m 是终端改变颜色的指令
    std::cout << "\033[1;" << (int)color << "m";
}

void Logger::Info(const std::string& message) {
    SetColor(GREEN);
    std::cout << "[INFO] " << message << "\033[0m" << std::endl;
}

void Logger::Warning(const std::string& message) {
    SetColor(YELLOW);
    std::cout << "[WARNING] " << message << "\033[0m" << std::endl;
}

void Logger::Error(const std::string& message) {
    SetColor(RED);
    std::cout << "[ERROR] " << message << "\033[0m" << std::endl;
}

void Logger::LogCollision(const Manifold& m) {
    SetColor(CYAN);
    std::cout << "[COLLISION] Depth: " << m.penetration
        << " | Normal: (" << m.normal.getX() << ", " << m.normal.getY() << ")"
        << "\033[0m" << std::endl;
}