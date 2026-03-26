#ifndef GRINDINGAPP_SRC_UTILS_VECTOR3D_H_
#define GRINDINGAPP_SRC_UTILS_VECTOR3D_H_

#include "GrindingUtilsExport.h"

namespace nl {
namespace utils {

// 三维向量，用于位置（mm）和姿态角（度）。
// 有意与 Eigen::Vector3d 区分：接口边界用此类型，内部算法用 Eigen。
struct Vector3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vector3d() = default;
    Vector3d(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    double  operator[](int i) const { return (&x)[i]; }
    double& operator[](int i)       { return (&x)[i]; }
};

} // namespace utils
} // namespace nl

#endif  // GRINDINGAPP_SRC_UTILS_VECTOR3D_H_
