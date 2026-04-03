#pragma once

#include "GrindingPlanningExport.h"
#include "IKinematicsService.h"

namespace planning {

// [Strategy] IKinematicsService 的 KDL 实现
// [Pimpl 精神] KDL/OCCT 类型全部在 .cpp 中，头文件零三方依赖
class GRINDING_PLANNING_EXPORT KdlKinematicsService : public IKinematicsService {
public:
    foundation::Result<std::vector<foundation::Pose>> ComputeFk(
        const domain::Robot& robot,
        const foundation::JointState& joints) override;

    foundation::Result<foundation::JointState> ComputeIk(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) override;

    foundation::Result<std::vector<foundation::JointState>> ComputeIkAll(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) override;
};

} // namespace planning
