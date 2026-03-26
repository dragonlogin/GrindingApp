#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_H_

#include <gp_Trsf.hxx>

namespace nl {
namespace occ {

// TCP 路径点：位姿 + 速度比例
// pose: Z 轴 = 曲面法线（工具接触方向），原点 = TCP 位置
struct Waypoint {
    gp_Trsf pose;
    double  speed_ratio = 1.0;
};

} // namespace occ
} // namespace nl

#endif  // GRINDINGAPP_SRC_OCC_WAYPOINT_H_
