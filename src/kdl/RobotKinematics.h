#ifndef GRINDINGAPP_SRC_KDL_ROBOT_KINEMATICS_H_
#define GRINDINGAPP_SRC_KDL_ROBOT_KINEMATICS_H_

#include <vector>
#include <gp_Trsf.hxx>

#include "GrindingKinematicsExport.h"
#include "RbXmlParser.h"

// Compute FK for all joints using KDL (Craig 1989 DH convention).
// joint_angles: array of 6 values in degrees (jog offsets, home offset handled internally).
// Returns one gp_Trsf per joint (world transform of joint frame, index 0 = Joint1).
GRINDING_KINEMATICS_EXPORT std::vector<gp_Trsf> ComputeFkKdl(const RbRobot& robot, const double joint_angles[6]);

#endif  // GRINDINGAPP_SRC_KDL_ROBOT_KINEMATICS_H_
