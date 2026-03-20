#pragma once
#include <gp_Trsf.hxx>
#include <QVector>
#include "RbXmlParser.h"

// 计算 Craig DH 变换矩阵（theta = joint_angle + offset, 单位 deg; a/d 单位 mm）
gp_Trsf DhTrsf(double theta_deg, double d, double a, double alpha_deg);

// 计算 RPY（RobWork: Rz(yaw)*Ry(pitch)*Rx(roll)）+ 平移
gp_Trsf RpyPosTrsf(const double rpy[3], const double pos[3]);

// 计算机器人在 home 姿态下各关节的世界变换（索引 0 = Joint1）
QVector<gp_Trsf> ComputeFkHome(const RbRobot& robot);
