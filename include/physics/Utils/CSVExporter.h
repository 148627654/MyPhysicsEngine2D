#pragma once
#include <fstream>
#include <string>
#include <vector>
#include "../Common/Vector2.h"

class CSVExporter {
public:
    CSVExporter(const std::string& filename) {
        m_file.open(filename);
        // 写入表头
        if (m_file.is_open()) {
            m_file << "Time,Pos_X,Pos_Y,Vel_X,Vel_Y,Angle\n";
        }
    }

    ~CSVExporter() {
        if (m_file.is_open()) m_file.close();
    }

    void AddData(float time, Vector2 pos, Vector2 vel, float angle) {
        if (m_file.is_open()) {
            m_file << time << ","
                << pos.getX() << "," << pos.getY() << ","
                << vel.getX() << "," << vel.getY() << ","
                << angle << "\n";
        }
    }

private:
    std::ofstream m_file;
};