#include "Logger.h"

void Logger::setColor(Color color) {
    // \033[1;...m 是终端改变颜色的指令
    std::cout << "\033[1;" << (int)color << "m";
}

void Logger::info(const std::string& message) {
    setColor(GREEN);
    std::cout << "[INFO] " << message << "\033[0m" << std::endl;
}

void Logger::warning(const std::string& message) {
    setColor(YELLOW);
    std::cout << "[WARNING] " << message << "\033[0m" << std::endl;
}

void Logger::error(const std::string& message) {
    setColor(RED);
    std::cout << "[ERROR] " << message << "\033[0m" << std::endl;
}

void Logger::logCollision(const Manifold& m) {
    setColor(CYAN);
    std::cout << "[COLLISION] Depth: " << m.penetration
        << " | Normal: (" << m.normal.getX() << ", " << m.normal.getY() << ")"
        << "\033[0m" << std::endl;
}