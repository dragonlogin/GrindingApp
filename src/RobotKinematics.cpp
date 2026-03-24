#include "RobotKinematics.h"

#include <cmath>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace {

constexpr double kDeg = M_PI / 180.0;

gp_Trsf KdlFrameToGpTrsf(const KDL::Frame& f)
{
    gp_Trsf t;
    t.SetValues(
        f.M(0, 0), f.M(0, 1), f.M(0, 2), f.p.x(),
        f.M(1, 0), f.M(1, 1), f.M(1, 2), f.p.y(),
        f.M(2, 0), f.M(2, 1), f.M(2, 2), f.p.z()
    );
    return t;
}

KDL::Chain BuildChain(const RbRobot& robot)
{
    KDL::Chain chain;
    for (const RbJoint& j : robot.joints) {
        // Craig 1989 DH convention: DH_Craig1989(a, alpha_rad, d, theta_offset_rad)
        // Home-position offset is baked into the zero frame; joint angle is additive.
        KDL::Frame f = KDL::Frame::DH_Craig1989(
            j.a,
            j.alpha_deg * kDeg,
            j.d,
            j.offset_deg * kDeg
        );
        chain.addSegment(KDL::Segment(
            j.name.toStdString(),
            KDL::Joint(KDL::Joint::RotZ),
            f
        ));
    }
    return chain;
}

}  // namespace

QVector<gp_Trsf> ComputeFkKdl(const RbRobot& robot, const double joint_angles[6])
{
    KDL::Chain chain = BuildChain(robot);
    KDL::ChainFkSolverPos_recursive solver(chain);

    int n = robot.joints.size();
    KDL::JntArray q(n);
    for (int i = 0; i < n; ++i)
        q(i) = (i < 6 ? joint_angles[i] : 0.0) * kDeg;

    QVector<gp_Trsf> result(n);
    for (int seg = 0; seg < n; ++seg) {
        KDL::Frame frame;
        // seg+1: compute FK up to and including segment index seg
        solver.JntToCart(q, frame, seg + 1);
        result[seg] = KdlFrameToGpTrsf(frame);
    }
    return result;
}
