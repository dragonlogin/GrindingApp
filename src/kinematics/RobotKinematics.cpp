#include "RobotKinematics.h"

#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace {

constexpr double kDeg = M_PI / 180.0;

// 将 Eigen 矩阵转换为 gp_Trsf
gp_Trsf EigenMatrixToGpTrsf(const Eigen::Matrix4d& mat)
{
    gp_Trsf trsf;
    trsf.SetValues(
        mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3),
        mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
        mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3)
    );
    return trsf;
}

// 计算单个 DH 变换矩阵
Eigen::Matrix4d ComputeDhMatrix(double theta_deg, double d, double a, double alpha_deg)
{
    double t = theta_deg * kDeg;
    double al = alpha_deg * kDeg;
    double ct = cos(t), st = sin(t);
    double ca = cos(al), sa = sin(al);

    Eigen::Matrix4d mat;
    mat << 
        ct,       -st,      0.0,   a,
        st * ca,   ct * ca, -sa,  -d * sa,
        st * sa,   ct * sa,  ca,   d * ca,
        0.0,       0.0,      0.0,  1.0;
    return mat;
}

}  // namespace

std::vector<gp_Trsf> ComputeFk(const RbRobot& robot, const double joint_angles[6])
{
    int n = static_cast<int>(robot.joints.size());
    std::vector<gp_Trsf> result(n);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (int i = 0; i < n; ++i) {
        const RbJoint& j = robot.joints[i];
        double theta = j.offset_deg + (i < 6 ? joint_angles[i] : 0.0);
        Eigen::Matrix4d dh = ComputeDhMatrix(theta, j.d, j.a, j.alpha_deg);

        T *= dh;
        result[i] = EigenMatrixToGpTrsf(T);
    }

    return result;
}