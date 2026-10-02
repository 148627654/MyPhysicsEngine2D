#include "CSVExporter.h"
#include <Body.h>

void CSVExporter::writeFrame(int frameIndex, const std::vector<Body*>& bodies)
{
	if (!m_file.is_open()) return;
    for (size_t i = 0; i < bodies.size(); ++i) {
        Body* b = bodies[i];

        // 写入数据：以逗号分隔
        m_file << frameIndex << ","
            << i << "," // 使用循环索引作为 BodyID
            << (b->getShape()->type == Shape::Type::type_Circle ? "Circle" : "Box") << ","
            << b->getPosition().getX() << ","
            << b->getPosition().getY() << ","
            << b->getRotation() << ","
            << b->getVelocity().getX() << ","
            << b->getVelocity().getY() << ","
            << b->getAngularVelocity() << "\n"; // 假设你把 angularVelocity 改为了 public 或有 Getter
    }
}
