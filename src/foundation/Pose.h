#pragma once

namespace foundation {

// Row-major 4x4 homogeneous transform matrix.
// Translation unit: meters.
struct Pose {
    double m[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    static Pose Identity()
    {
        return Pose{};
    }
};

} // namespace foundation
