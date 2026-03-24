#ifndef GRINDINGAPP_SRC_ROBOT_KINEMATICS_H_
#define GRINDINGAPP_SRC_ROBOT_KINEMATICS_H_

#include <QVector>
#include <gp_Trsf.hxx>
#include "RbXmlParser.h"

// Compute FK for all joints using KDL (Craig 1989 DH convention).
// joint_angles: array of 6 values in degrees (jog offsets, home offset handled internally).
// Returns one gp_Trsf per joint (world transform of joint frame, index 0 = Joint1).
QVector<gp_Trsf> ComputeFkKdl(const RbRobot& robot, const double joint_angles[6]);

#endif  // GRINDINGAPP_SRC_ROBOT_KINEMATICS_H_
