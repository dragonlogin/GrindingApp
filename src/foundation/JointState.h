#pragma once

#include <vector>

namespace foundation {

// Joint values in degrees on module boundaries.
struct JointState {
    std::vector<double> values_deg;

    int size() const
    {
        return static_cast<int>(values_deg.size());
    }

    bool empty() const
    {
        return values_deg.empty();
    }

    double operator[](int index) const
    {
        return values_deg[index];
    }

    double& operator[](int index)
    {
        return values_deg[index];
    }
};

} // namespace foundation
