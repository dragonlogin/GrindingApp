#include "WaypointPlanarAlgo.h"

#include <cfloat>

#include "Conversions.h"

#include <BRepAlgoAPI_Section.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>

namespace nl {
namespace occ {

std::vector<Waypoint> WaypointPlanarAlgo::Generate(
    const TopoDS_Face& face,
    const WaypointConfig& config)
{
    std::vector<Waypoint> result;
    if (face.IsNull() || config.num_slices < 2)
        return result;

    // Compute bounding box
    Bnd_Box bbox;
    BRepBndLib::Add(face, bbox);
    double xmin, ymin, zmin, xmax, ymax, zmax;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

    // Project corners onto cut direction to get range
    gp_Dir normal = config.cut_normal;
    gp_Pnt corners[8] = {
        {xmin, ymin, zmin}, {xmax, ymin, zmin},
        {xmin, ymax, zmin}, {xmax, ymax, zmin},
        {xmin, ymin, zmax}, {xmax, ymin, zmax},
        {xmin, ymax, zmax}, {xmax, ymax, zmax}
    };

    double d_min = DBL_MAX, d_max = -DBL_MAX;
    for (int i = 0; i < 8; ++i) {
        double d = gp_Vec(normal).Dot(gp_Vec(corners[i].XYZ()));
        d_min = std::min(d_min, d);
        d_max = std::max(d_max, d);
    }

    double slice_step = (d_max - d_min) / config.num_slices;

    for (int i = 0; i <= config.num_slices; ++i) {
        double d = d_min + i * slice_step;

        // Create cut plane: normal dot P = d
        gp_Pnt plane_origin(normal.X() * d, normal.Y() * d, normal.Z() * d);
        gp_Pln cut_plane(plane_origin, normal);

        // Intersect face with plane
        BRepAlgoAPI_Section section(face, cut_plane);
        section.ComputePCurveOn1(true);
        section.Approximation(true);
        section.Build();

        if (!section.IsDone()) continue;

        TopoDS_Shape section_result = section.Shape();

        // Traverse intersection edges
        for (TopExp_Explorer exp(section_result, TopAbs_EDGE); exp.More(); exp.Next()) {
            TopoDS_Edge edge = TopoDS::Edge(exp.Current());
            BRepAdaptor_Curve curve(edge);

            double t_min = curve.FirstParameter();
            double t_max = curve.LastParameter();

            int num_samples = 20;
            double t_step = (t_max - t_min) / num_samples;

            for (int j = 0; j <= num_samples; ++j) {
                double t = t_min + j * t_step;
                gp_Pnt pos = curve.Value(t);

                // Build TCP frame: Z = cut normal
                gp_Ax3 ax3(pos, normal);
                gp_Trsf trsf;
                trsf.SetTransformation(ax3);

                // Apply approach distance
                if (config.approach_dist > 0) {
                    gp_Vec offset(normal);
                    offset.Scale(config.approach_dist);
                    gp_XYZ new_pos = pos.XYZ() + offset.XYZ();
                    trsf.SetTranslationPart(gp_Vec(new_pos));
                }

                result.push_back({foundation::ToPose(trsf), 1.0});
            }
        }
    }

    return result;
}

} // namespace occ
} // namespace nl
