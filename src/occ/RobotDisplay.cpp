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

void TrsfToRpyPos(const gp_Trsf& trsf, Vector3d& rpy, Vector3d& pos)
{
    const gp_XYZ& t = trsf.TranslationPart();
    pos = {t.X(), t.Y(), t.Z()};

    const gp_Mat& R = trsf.VectorialPart();
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // R(2,0) = -sin(pitch)
    // gp_Mat is 1-indexed: Value(r,c) = R(r-1, c-1)
    double sp = -R.Value(3, 1);  // -R(2,0) = sin(pitch)
    if (sp > 1.0 - 1e-10) {
        // Gimbal lock: pitch = +90, roll = 0, yaw = atan2(R(0,1), R(1,1))
        rpy[1] = 90.0;
        rpy[0] = atan2(R.Value(1, 2), R.Value(2, 2)) / kDeg;
        rpy[2] = 0.0;
    } else if (sp < -1.0 + 1e-10) {
        // Gimbal lock: pitch = -90, roll = 0, yaw = atan2(-R(0,1), R(1,1))
        rpy[1] = -90.0;
        rpy[0] = atan2(-R.Value(1, 2), R.Value(2, 2)) / kDeg;
        rpy[2] = 0.0;
    } else {
        rpy[1] = asin(sp) / kDeg;                                         // pitch
        rpy[0] = atan2(R.Value(2, 1), R.Value(1, 1)) / kDeg;             // yaw  = atan2(R(1,0), R(0,0))
        rpy[2] = atan2(R.Value(3, 2), R.Value(3, 3)) / kDeg;             // roll = atan2(R(2,1), R(2,2))
    }
}

} // namespace occ
} // namespace nl
