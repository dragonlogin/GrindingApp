#ifndef GRINDINGAPP_SRC_OCC_SURFACE_WAYPOINT_GEN_H_
#define GRINDINGAPP_SRC_OCC_SURFACE_WAYPOINT_GEN_H_

#include <vector>

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include "GrindingOccExport.h"
#include "Waypoint.h"

namespace nl {
namespace occ {

// 从 shape 中选取面积最大的 Face
GRINDING_OCC_EXPORT
TopoDS_Face LargestFace(const TopoDS_Shape& shape);

// 在 face 上按 u_steps x v_steps 网格生成 TCP 路径点（蛇形行切）
// approach_dist: 工具沿法线方向偏移量（mm），0 = 贴面
GRINDING_OCC_EXPORT
std::vector<Waypoint> GenerateGridWaypoints(
    const TopoDS_Face& face,
    int u_steps,
    int v_steps,
    double approach_dist = 0.0);

} // namespace occ
} // namespace nl

#endif  // GRINDINGAPP_SRC_OCC_SURFACE_WAYPOINT_GEN_H_
