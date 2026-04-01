#include "WaypointGridAlgo.h"

#include <cmath>

#include "Conversions.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepLProp_SLProps.hxx>
#include <TopAbs_Orientation.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace nl {
namespace occ {

std::vector<Waypoint> WaypointGridAlgo::Generate(
    const TopoDS_Face& face,
    const WaypointConfig& config)
{
    std::vector<Waypoint> result;
    if (face.IsNull() || config.u_steps < 2 || config.v_steps < 2)
        return result;

    BRepAdaptor_Surface adaptor(face);
    double u_min = adaptor.FirstUParameter();
    double u_max = adaptor.LastUParameter();
    double v_min = adaptor.FirstVParameter();
    double v_max = adaptor.LastVParameter();

    bool flip_normal = (face.Orientation() == TopAbs_REVERSED);

    for (int iv = 0; iv < config.v_steps; ++iv) {
        double v = v_min + (v_max - v_min) * iv / (config.v_steps - 1);

        bool reverse_row = (iv % 2 == 1);

        for (int iu_idx = 0; iu_idx < config.u_steps; ++iu_idx) {
            int iu = reverse_row ? (config.u_steps - 1 - iu_idx) : iu_idx;
            double u = u_min + (u_max - u_min) * iu / (config.u_steps - 1);

            BRepLProp_SLProps props(adaptor, u, v, 1, 1e-6);
            if (!props.IsNormalDefined())
                continue;

            gp_Pnt pt = props.Value();
            gp_Dir normal = props.Normal();
            if (flip_normal)
                normal.Reverse();

            if (config.approach_dist != 0.0)
                pt.Translate(gp_Vec(normal) * config.approach_dist);

            gp_Dir z_dir = -normal;
            gp_Dir x_dir;
            if (props.IsTangentUDefined()) {
                props.TangentU(x_dir);
            } else {
                gp_Dir ref(0, 0, 1);
                if (std::abs(z_dir.Dot(ref)) > 0.9)
                    ref = gp_Dir(1, 0, 0);
                x_dir = ref.Crossed(z_dir);
            }

            gp_Dir y_dir = z_dir.Crossed(x_dir);
            x_dir = y_dir.Crossed(z_dir);

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
