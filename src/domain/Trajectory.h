#pragma once

#include <vector>

#include "Ids.h"
#include "Pose.h"
#include "JointState.h"

namespace domain {

enum class MoveType {
    kMoveJ,
    kMoveL
};

enum class TrajectoryPointStatus {
    kOk,
    kIkFailed,
    kJointJump,
    kCollision
};


struct TrajectoryPoint {
    foundation::Pose tcp_pose;
    foundation::JointState joint_state;
    MoveType move_type = MoveType::kMoveL;
    TrajectoryPointStatus status = TrajectoryPointStatus::kOk;
    int waypoint_index = -1;
};

struct Trajectory {
    foundation::TrajectoryId id;
    std::vector<TrajectoryPoint> points;

    bool HasErrors() const
    {
        for (const auto& p : points) {
            if (p.status != TrajectoryPointStatus::kOk) {
                return true;
            }
        }
        return false;
    }

    int ErrorCount() const
    {
        int count = 0;
        for (const auto& p : points) {
            if (p.status != TrajectoryPointStatus::kOk) {
                ++count;
            }
        }
        return count;
    }
};

} // namespace domain