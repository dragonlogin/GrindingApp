#ifndef GRINDINGAPP_SRC_KINEMATICS_KDLSOLVER_H_
#define GRINDINGAPP_SRC_KINEMATICS_KDLSOLVER_H_

#include "GrindingKinematicsExport.h"
#include "IKinematicsSolver.h"

namespace nl {
namespace kinematics {

class GRINDING_KINEMATICS_EXPORT KdlSolver : public IKinematicsSolver {
public:
    std::vector<gp_Trsf> ComputeFk(
        const nl::core::RbRobot& robot,
        const nl::utils::Q& joint_angles) override;

    bool ComputeIk(
        const nl::core::RbRobot& robot,
        const gp_Trsf& target,
        const nl::utils::Q& init,
        nl::utils::Q& out) override;
};

} // namespace kinematics
} // namespace nl

#endif  // GRINDINGAPP_SRC_KINEMATICS_KDLSOLVER_H_
