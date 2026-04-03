#pragma once

#include <vector>

#include "foundation/JointState.h"
#include "foundation/Pose.h"
#include "foundation/Result.h"
#include "domain/Robot.h"

namespace planning {

// [Strategy] 运动学抽象接口，隔离底层求解器实现
class IKinematicsService {
public:
    virtual ~IKinematicsService() = default;

    // 正向运动学：返回每个关节的世界坐标变换（index 0 = Joint1）
    virtual foundation::Result<std::vector<foundation::Pose>> ComputeFk(
        const domain::Robot& robot,
        const foundation::JointState& joints) = 0;

    // 逆向运动学：返回最优单解（关节空间距离 seed 最近）
    virtual foundation::Result<foundation::JointState> ComputeIk(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) = 0;

    // 逆向运动学：返回所有解（最多 8 个，按距 seed 距离排序）
    virtual foundation::Result<std::vector<foundation::JointState>> ComputeIkAll(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) = 0;
};

} // namespace planning
