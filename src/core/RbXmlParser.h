#ifndef GRINDINGAPP_SRC_RBXMLPARSER_H_
#define GRINDINGAPP_SRC_RBXMLPARSER_H_

#include <string>
#include <vector>

#include "GrindingCoreExport.h"
#include "Vector3d.h"

namespace nl {
namespace core {

struct RbJoint {
    std::string name;
    double alpha_deg;
    double a;           // mm
    double d;           // mm
    double offset_deg;  // home position offset
};

struct RbDrawable {
    std::string name;
    std::string ref_joint;  // "Robot_Base" or "Joint1".."Joint6"
    utils::Vector3d rpy;          // degrees: yaw, pitch, roll
    utils::Vector3d pos;          // mm
    std::string mesh_file;  // absolute path (with .stl extension)
};

struct RbRobot {
    std::string name;
    std::string source_path;  // absolute robot model path, now typically .urdf
    std::vector<RbJoint>    joints;
    std::vector<RbDrawable> drawables;
};

class GRINDING_CORE_EXPORT RbXmlParser {
public:
    static RbRobot Parse(const std::string& xml_path);
};

} // namespace core
} // namespace nl

#endif  // GRINDINGAPP_SRC_RBXMLPARSER_H_
