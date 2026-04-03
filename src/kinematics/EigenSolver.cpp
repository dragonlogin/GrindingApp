#include "EigenSolver.h"
#include "KdlSolver.h"

namespace nl {
namespace kinematics {

std::vector<gp_Trsf> EigenSolver::ComputeFk(
    const domain::Robot& robot,
    const nl::utils::Q& joint_angles)
{
    KdlSolver solver;
    return solver.ComputeFk(robot, joint_angles);
}

bool EigenSolver::ComputeIk(
    const domain::Robot& robot,
    const gp_Trsf& target,
    const nl::utils::Q& init,
    nl::utils::Q& out)
{
    KdlSolver solver;
    return solver.ComputeIk(robot, target, init, out);
}

} // namespace kinematics
} // namespace nl
