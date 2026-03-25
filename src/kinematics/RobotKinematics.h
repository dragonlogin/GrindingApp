#ifndef GRINDINGAPP_SRC_KINEMATICS_ROBOT_KINEMATICS_H_
#define GRINDINGAPP_SRC_KINEMATICS_ROBOT_KINEMATICS_H_

#include <vector>
#include <gp_Trsf.hxx>

#include "GrindingKinematicsExport.h"
#include "RbXmlParser.h"

namespace nl {
namespace kinematics {

// Compute FK for all joints using Eigen3 (Craig 1989 DH convention).
// joint_angles: 6 jog angles in degrees (DH offset handled internally).
// Returns one gp_Trsf per joint (world transform, index 0 = Joint1).
GRINDING_KINEMATICS_EXPORT std::vector<gp_Trsf> ComputeFk(
    const nl::core::RbRobot& robot, const double joint_angles[6]);

// Compute IK using Jacobian pseudo-inverse (SVD, damped least squares).
// target:      desired end-effector pose (joint6 world transform).
// init_angles: initial guess [6] in degrees (jog angles, without DH offset).
// out_angles:  solution [6] in degrees (jog angles). Written even on failure.
// Returns true if position error < 0.01 mm and rotation error < 1e-4 rad.
GRINDING_KINEMATICS_EXPORT bool ComputeIk(
    const nl::core::RbRobot& robot, const gp_Trsf& target,
    const double init_angles[6], double out_angles[6]);

} // namespace kinematics
} // namespace nl

#endif  // GRINDINGAPP_SRC_KINEMATICS_ROBOT_KINEMATICS_H_
