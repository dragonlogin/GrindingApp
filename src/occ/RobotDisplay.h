#ifndef GRINDINGAPP_SRC_OCC_ROBOT_DISPLAY_H_
#define GRINDINGAPP_SRC_OCC_ROBOT_DISPLAY_H_

#include <vector>
#include <gp_Trsf.hxx>

#include "GrindingOccExport.h"
#include "RbXmlParser.h"

namespace nl {
namespace occ {

// Compute Craig DH transform matrix (theta = joint_angle + offset, degrees; a/d in mm).
GRINDING_OCC_EXPORT gp_Trsf DhTrsf(double theta_deg, double d, double a, double alpha_deg);

// Compute RPY (RobWork: Rz(yaw)*Ry(pitch)*Rx(roll)) + translation.
GRINDING_OCC_EXPORT gp_Trsf RpyPosTrsf(const double rpy[3], const double pos[3]);

// Compute world transforms for all joints at home pose (index 0 = Joint1).
GRINDING_OCC_EXPORT std::vector<gp_Trsf> ComputeFkHome(const nl::core::RbRobot& robot);

} // namespace occ
} // namespace nl

#endif  // GRINDINGAPP_SRC_OCC_ROBOT_DISPLAY_H_
