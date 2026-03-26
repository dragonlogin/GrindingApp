#ifndef GRINDINGAPP_SRC_KINEMATICS_IKINEMATICS_SOLVER_H_
#define GRINDINGAPP_SRC_KINEMATICS_IKINEMATICS_SOLVER_H_

#include <vector>
#include <gp_Trsf.hxx>

#include "GrindingKinematicsExport.h"

namespace nl { namespace core { struct RbRobot; } }
namespace nl { namespace utils { class Q; } }

namespace nl {
namespace kinematics {

// Abstract kinematics interface.
// All angle units: degrees (jog angles, without DH offset).
class GRINDING_KINEMATICS_EXPORT IKinematicsSolver {
public:
    virtual ~IKinematicsSolver() = default;

    // Forward kinematics: returns one world gp_Trsf per joint (index 0 = Joint1).
    virtual std::vector<gp_Trsf> ComputeFk(
        const nl::core::RbRobot& robot,
        const nl::utils::Q& joint_angles) = 0;

    // Inverse kinematics.
    // target:     desired end-effector world transform.
    // init:       initial guess in degrees.
    // out:        solution in degrees (written even on failure).
    // Returns true if converged within tolerance.
    virtual bool ComputeIk(
        const nl::core::RbRobot& robot,
        const gp_Trsf& target,
        const nl::utils::Q& init,
        nl::utils::Q& out) = 0;
};

} // namespace kinematics
} // namespace nl

#endif  // GRINDINGAPP_SRC_KINEMATICS_IKINEMATICS_SOLVER_H_
