#include "EigenSolver.h"
#include "RobotKinematics.h"

namespace nl {
namespace kinematics {

std::vector<gp_Trsf> EigenSolver::ComputeFk(
    const nl::core::RbRobot& robot,
    const nl::utils::Q& joint_angles)
{
    return ::nl::kinematics::ComputeFk(robot, joint_angles);
}

bool EigenSolver::ComputeIk(
    const nl::core::RbRobot& robot,
    const gp_Trsf& target,
    const nl::utils::Q& init,
    nl::utils::Q& out)
{
    return ::nl::kinematics::ComputeIk(robot, target, init, out);
}

} // namespace kinematics
} // namespace nl
