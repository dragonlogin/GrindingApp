#include "RobotDisplay.h"
#include <cmath>
#include <vector>

using nl::core::RbRobot;
using nl::core::RbJoint;
using nl::utils::Vector3d;

namespace nl {
namespace occ {

namespace {
constexpr double kDeg = M_PI / 180.0;
} // namespace

gp_Trsf DhTrsf(double theta_deg, double d, double a, double alpha_deg)
{
    double t  = theta_deg * kDeg;
    double al = alpha_deg * kDeg;
    double ct = cos(t), st = sin(t);
    double ca = cos(al), sa = sin(al);

    gp_Trsf trsf;
    // Craig Standard DH: Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    trsf.SetValues(
        ct,       -st,      0.0,   a,
        st * ca,   ct * ca, -sa,  -d * sa,
        st * sa,   ct * sa,  ca,   d * ca
    );
    return trsf;
}

gp_Trsf RpyPosTrsf(const Vector3d& rpy, const Vector3d& pos)
{
    double cy = cos(rpy[0] * kDeg), sy = sin(rpy[0] * kDeg);  // rpy[0] = Z (yaw)
    double cp = cos(rpy[1] * kDeg), sp = sin(rpy[1] * kDeg);  // rpy[1] = Y (pitch)
    double cr = cos(rpy[2] * kDeg), sr = sin(rpy[2] * kDeg);  // rpy[2] = X (roll)

    // R = Rz(rpy[0]) * Ry(rpy[1]) * Rx(rpy[2])
    gp_Trsf trsf;
    trsf.SetValues(
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, pos[0],
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, pos[1],
        -sp,     cp * sr,                cp * cr,                 pos[2]
    );
    return trsf;
}

std::vector<gp_Trsf> ComputeFkHome(const RbRobot& robot)
{
    std::vector<gp_Trsf> fk(robot.joints.size());
    for (int i = 0; i < static_cast<int>(robot.joints.size()); ++i) {
        const RbJoint& j = robot.joints[i];
        gp_Trsf dh = DhTrsf(j.offset_deg, j.d, j.a, j.alpha_deg);
        if (i == 0) {
            fk[i] = dh;
        } else {
            fk[i] = fk[i - 1];
            fk[i].Multiply(dh);
        }
    }
    return fk;
}

} // namespace occ
} // namespace nl
