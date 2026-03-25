#ifndef GRINDINGAPP_SRC_RBXMLPARSER_H_
#define GRINDINGAPP_SRC_RBXMLPARSER_H_

#include <string>
#include <vector>

#include "GrindingCoreExport.h"

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
    double rpy[3];          // degrees: roll, pitch, yaw
    double pos[3];          // mm
    std::string mesh_file;  // absolute path (with .stl extension)
};

struct RbRobot {
    std::string name;
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
