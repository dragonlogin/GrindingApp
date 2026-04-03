#pragma once

#include <string>
#include <vector>

namespace domain {

struct RobotJoint {
    std::string name;
    double alpha_deg;
    double a_mm;       // 连杆长度，毫米
    double d_mm;       // 连杆偏置，毫米
    double offset_deg; // home 位置偏移
};

// [Value Object] 纯运动学领域模型，不含渲染/网格数据
struct Robot {
    std::string name;
    std::string source_path;  // URDF 路径，KdlKinematicsService 用于读关节限位
    std::vector<RobotJoint> joints;
};

} // namespace domain