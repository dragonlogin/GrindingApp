#pragma once

namespace planning {

struct PlanningRequest {
    double approach_dist = 50.0;
    int movej_steps = 50;
    int movel_steps_per_seg = 10;
    double joint_jump_threshold = 30.0;
};

} // namespace planning