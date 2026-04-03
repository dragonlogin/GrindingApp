#pragma once

#include <vector>
#include <gp_Trsf.hxx>

#include "GrindingKinematicsExport.h"
#include "domain/Robot.h"

namespace nl { namespace utils { class Q; } }

namespace nl {
namespace kinematics {

// 低层运动学接口，gp_Trsf/Q 为内部计算类型
class GRINDING_KINEMATICS_EXPORT IKinematicsSolver {
public:
    virtual ~IKinematicsSolver() = default;

    // 正向运动学：返回每个关节的世界变换（index 0 = Joint1）
    virtual std::vector<gp_Trsf> ComputeFk(
        const domain::Robot& robot,
        const nl::utils::Q& joint_angles) = 0;

    // 逆向运动学，返回 true 表示收敛
    virtual bool ComputeIk(
        const domain::Robot& robot,
        const gp_Trsf& target,
        const nl::utils::Q& init,
        nl::utils::Q& out) = 0;
};

} // namespace kinematics
} // namespace nl
