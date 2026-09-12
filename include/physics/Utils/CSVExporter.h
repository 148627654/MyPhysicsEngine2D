#pragma once
#include <fstream>
#include <string>
#include <vector>
#include "../Common/Vector2.h"
#include "../Dynamics/Body.h"
#include "../Utils/Logger.h"
class CSVExporter {
public:
	CSVExporter(const std::string& filename) {
		// 尝试打开文件
		m_file.open(filename);

		if ( !m_file.is_open( ) ) {
			// 如果打开失败，通常是文件夹不存在
			Logger::Error("Failed to open/create CSV file: " + filename);
			Logger::Warning("Please ensure the directory exists (e.g., create an 'output' folder).");
		}
		else {
			// 成功打开（或创建），写入表头
			m_file << "Frame,BodyID,Shape,PosX,PosY,Angle,VelX,VelY,AngVel\n";
			Logger::Info("CSV File initialized: " + filename);
		}
	}

    ~CSVExporter() {
        if (m_file.is_open()) m_file.close();
    }
void WriteFrame(int frameIndex , const std::vector<Body*>& bodies);
private:
    std::ofstream m_file;
};