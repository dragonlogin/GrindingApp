#ifndef GRINDINGAPP_SRC_ROBOT_DISPLAY_H_
#define GRINDINGAPP_SRC_ROBOT_DISPLAY_H_

#include <gp_Trsf.hxx>
#include <QVector>
#include "RbXmlParser.h"

// Compute Craig DH transform matrix (theta = joint_angle + offset, degrees; a/d in mm).
gp_Trsf DhTrsf(double theta_deg, double d, double a, double alpha_deg);

// Compute RPY (RobWork: Rz(yaw)*Ry(pitch)*Rx(roll)) + translation.
gp_Trsf RpyPosTrsf(const double rpy[3], const double pos[3]);

// Compute world transforms for all joints at home pose (index 0 = Joint1).
QVector<gp_Trsf> ComputeFkHome(const RbRobot& robot);

#endif  // GRINDINGAPP_SRC_ROBOT_DISPLAY_H_
