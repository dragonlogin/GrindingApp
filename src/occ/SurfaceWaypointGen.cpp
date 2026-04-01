#include "SurfaceWaypointGen.h"

#include <cmath>

#include "Conversions.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace nl {
namespace occ {

TopoDS_Face LargestFace(const TopoDS_Shape& shape)
{
    TopoDS_Face largest;
    double max_area = -1.0;

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        double area = props.Mass();
        if (area > max_area) {
            max_area = area;
            largest  = face;
        }
    }
    return largest;
}

std::vector<Waypoint> GenerateGridWaypoints(
    const TopoDS_Face& face,
    int u_steps,
    int v_steps,
    double approach_dist)
{
    std::vector<Waypoint> result;
    if (face.IsNull() || u_steps < 2 || v_steps < 2)
        return result;

    BRepAdaptor_Surface adaptor(face);
    double u_min = adaptor.FirstUParameter();
    double u_max = adaptor.LastUParameter();
    double v_min = adaptor.FirstVParameter();
    double v_max = adaptor.LastVParameter();

    // Face orientation 决定法线是否需要翻转
    bool flip_normal = (face.Orientation() == TopAbs_REVERSED);

    for (int iv = 0; iv < v_steps; ++iv) {
        double v = v_min + (v_max - v_min) * iv / (v_steps - 1);

        // 蛇形（boustrophedon）：偶数行正向，奇数行反向
        bool reverse_row = (iv % 2 == 1);

        for (int iu_idx = 0; iu_idx < u_steps; ++iu_idx) {
            int    iu = reverse_row ? (u_steps - 1 - iu_idx) : iu_idx;
            double u  = u_min + (u_max - u_min) * iu / (u_steps - 1);

            BRepLProp_SLProps props(adaptor, u, v, 1, 1e-6);
            if (!props.IsNormalDefined())
                continue;

            gp_Pnt pt     = props.Value();
            gp_Dir normal = props.Normal();
            if (flip_normal)
                normal.Reverse();

            // 沿法线偏移
            if (approach_dist != 0.0)
                pt.Translate(gp_Vec(normal) * approach_dist);

            // 构造坐标系：Z = 法线，X = u 方向切线（或回退方向）
            gp_Dir z_dir = normal;
            gp_Dir x_dir;
            if (props.IsTangentUDefined()) {
                props.TangentU(x_dir);
            } else {
                // 回退：取一个不与法线平行的参考方向，叉积得到 x
                gp_Dir ref(0, 0, 1);
                if (std::abs(z_dir.Dot(ref)) > 0.9)
                    ref = gp_Dir(1, 0, 0);
                x_dir = ref.Crossed(z_dir);
            }

            // 重正交化：Y = Z × X，X = Y × Z
            gp_Dir y_dir = z_dir.Crossed(x_dir);
            x_dir        = y_dir.Crossed(z_dir);

            // 构造 4x4 位姿矩阵（列向量：X Y Z T）
            gp_Trsf pose;
            pose.SetValues(
                x_dir.X(), y_dir.X(), z_dir.X(), pt.X(),
                x_dir.Y(), y_dir.Y(), z_dir.Y(), pt.Y(),
                x_dir.Z(), y_dir.Z(), z_dir.Z(), pt.Z());

            result.push_back({foundation::ToPose(pose), 1.0});
        }
    }
    return result;
}

} // namespace occ
} // namespace nl
