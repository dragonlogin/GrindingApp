#pragma once

#include "GrindingKinematicsExport.h"
#include "IKinematicsSolver.h"

namespace nl {
namespace kinematics {

// Kinematics solver using Eigen3.
// FK:  Craig DH matrix chain.
// IK:  Jacobian pseudo-inverse (SVD damped least squares), 300 iterations.
class GRINDING_KINEMATICS_EXPORT EigenSolver : public IKinematicsSolver {
public:
    std::vector<gp_Trsf> ComputeFk(
        const domain::Robot& robot,
        const nl::utils::Q& joint_angles) override;

    bool ComputeIk(
        const domain::Robot& robot,
        const gp_Trsf& target,
        const nl::utils::Q& init,
        nl::utils::Q& out) override;
};

} // namespace kinematics
} // namespace nl
