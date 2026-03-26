#ifndef GRINDINGAPP_SRC_UTILS_Q_H_
#define GRINDINGAPP_SRC_UTILS_Q_H_

#include <initializer_list>
#include <vector>

#include "GrindingUtilsExport.h"

namespace nl {
namespace utils {

// Robot joint configuration (degrees). Naming follows RobWork convention.
class GRINDING_UTILS_EXPORT Q {
public:
    explicit Q(int n = 6, double val = 0.0);
    Q(std::initializer_list<double> vals);

    int    size() const                   { return static_cast<int>(values_.size()); }
    double  operator[](int i) const        { return values_[i]; }
    double& operator[](int i)              { return values_[i]; }
    const double* data() const             { return values_.data(); }

private:
    std::vector<double> values_;
};

} // namespace utils
} // namespace nl

#endif  // GRINDINGAPP_SRC_UTILS_Q_H_
