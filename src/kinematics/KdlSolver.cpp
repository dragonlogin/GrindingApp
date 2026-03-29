#include "KdlSolver.h"

#include <cmath>

#include <QDebug>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "KdlChainBuilder.h"
#include "Q.h"

namespace nl {
namespace kinematics {

namespace {

constexpr double kDeg = M_PI / 180.0;

gp_Trsf KdlFrameToGpTrsf(const KDL::Frame& frame)
{
    gp_Trsf trsf;
    trsf.SetValues(
        frame.M(0, 0), frame.M(0, 1), frame.M(0, 2), frame.p.x(),
        frame.M(1, 0), frame.M(1, 1), frame.M(1, 2), frame.p.y(),
        frame.M(2, 0), frame.M(2, 1), frame.M(2, 2), frame.p.z());
    return trsf;
}

KDL::Frame GpTrsfToKdlFrame(const gp_Trsf& trsf)
{
    const gp_Mat& rot = trsf.VectorialPart();
    const gp_XYZ& pos = trsf.TranslationPart();

    return KDL::Frame(
        KDL::Rotation(
            rot.Value(1, 1), rot.Value(1, 2), rot.Value(1, 3),
            rot.Value(2, 1), rot.Value(2, 2), rot.Value(2, 3),
            rot.Value(3, 1), rot.Value(3, 2), rot.Value(3, 3)),
        KDL::Vector(pos.X(), pos.Y(), pos.Z()));
}

KDL::JntArray ToKdlJoints(const nl::utils::Q& joint_angles)
{
    KDL::JntArray q(joint_angles.size());
    for (int i = 0; i < joint_angles.size(); ++i)
        q(i) = joint_angles[i] * kDeg;
    return q;
}

void FromKdlJoints(const KDL::JntArray& q, nl::utils::Q& joint_angles)
{
    for (unsigned int i = 0; i < q.rows() && i < static_cast<unsigned int>(joint_angles.size()); ++i)
        joint_angles[i] = q(i) / kDeg;
}

} // namespace

std::vector<gp_Trsf> KdlSolver::ComputeFk(
    const nl::core::RbRobot& robot,
    const nl::utils::Q& joint_angles)
{
    KDL::Chain chain = BuildKdlChain(robot, false);
    KDL::ChainFkSolverPos_recursive solver(chain);

    KDL::JntArray q = ToKdlJoints(joint_angles);
    std::vector<gp_Trsf> result;
    result.reserve(robot.joints.size());
    for (int i = 0; i < static_cast<int>(robot.joints.size()); ++i) {
        KDL::Frame frame;
        const int segment_nr = (i + 1) * 2;
        if (solver.JntToCart(q, frame, segment_nr) < 0)
            return {};
        result.push_back(KdlFrameToGpTrsf(frame));
    }
    return result;
}

bool KdlSolver::ComputeIk(
    const nl::core::RbRobot& robot,
    const gp_Trsf& target,
    const nl::utils::Q& init,
    nl::utils::Q& out)
{
    KDL::Chain chain = BuildKdlChain(robot, false);
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv vel_solver(chain);
    KDL::JntArray q_min;
    KDL::JntArray q_max;
    if (!BuildKdlJointLimits(robot, q_min, q_max))
        return false;
    KDL::ChainIkSolverPos_NR_JL pos_solver(
        chain, q_min, q_max, fk_solver, vel_solver, 300, 1e-6);

    KDL::JntArray q_init = ToKdlJoints(init);
    KDL::JntArray q_out(chain.getNrOfJoints());
    const KDL::Frame kdl_target = GpTrsfToKdlFrame(target);
    const int rc = pos_solver.CartToJnt(q_init, kdl_target, q_out);

    if (rc < 0) {
        // --- 诊断：打印目标位姿 + 初始角 ---
        const double px = kdl_target.p.x();
        const double py = kdl_target.p.y();
        const double pz = kdl_target.p.z();
        double roll, pitch, yaw;
        kdl_target.M.GetRPY(roll, pitch, yaw);
        constexpr double kRad2Deg = 180.0 / M_PI;
        qDebug() << "[IK FAIL] rc=" << rc
                 << " target pos=(" << px << "," << py << "," << pz << ")"
                 << " rpy=(" << roll*kRad2Deg << "," << pitch*kRad2Deg << "," << yaw*kRad2Deg << ") deg"
                 << " dist_from_base=" << std::sqrt(px*px + py*py + pz*pz)
                 << " init=[" << (init.size()>0?init[0]:0) << ","
                              << (init.size()>1?init[1]:0) << ","
                              << (init.size()>2?init[2]:0) << ","
                              << (init.size()>3?init[3]:0) << ","
                              << (init.size()>4?init[4]:0) << ","
                              << (init.size()>5?init[5]:0) << "]";
    }

    if (out.size() != static_cast<int>(chain.getNrOfJoints()))
        out = nl::utils::Q(static_cast<int>(chain.getNrOfJoints()), 0.0);
    FromKdlJoints(q_out, out);
    return rc >= 0;
}

} // namespace kinematics
} // namespace nl
