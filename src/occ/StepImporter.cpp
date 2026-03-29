#include "StepImporter.h"

#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <gp_GTrsf.hxx>

#include <iostream>

namespace nl {
namespace occ {

namespace {

constexpr double kWorkpieceMmToM = 0.001;
constexpr double kWorkpieceMeshDeflectionM = 0.0001;  // 0.1 mm

TopoDS_Shape ScaleWorkpieceToSceneUnits(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        return shape;

    gp_GTrsf scale_trsf;
    scale_trsf.SetValue(1, 1, kWorkpieceMmToM);
    scale_trsf.SetValue(2, 2, kWorkpieceMmToM);
    scale_trsf.SetValue(3, 3, kWorkpieceMmToM);
    return BRepBuilderAPI_GTransform(shape, scale_trsf, Standard_True).Shape();
}

} // namespace

TopoDS_Shape StepImporter::Load(const std::string& file_path, int* face_count)
{
    STEPControl_Reader reader;
    const IFSelect_ReturnStatus status = reader.ReadFile(file_path.c_str());

    if (status != IFSelect_RetDone) {
        std::cerr << "[StepImporter] Failed to read: " << file_path << "\n";
        return TopoDS_Shape();
    }

    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();

    // Project convention: workpiece CAD is authored in millimetres, while the
    // URDF-driven robot/tool scene uses metres.
    shape = ScaleWorkpieceToSceneUnits(shape);

    // Preserve the original 0.1 mm tessellation quality after scaling.
    BRepMesh_IncrementalMesh(shape, kWorkpieceMeshDeflectionM);

    int count = 0;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
        ++count;

    std::cout << "[StepImporter] Faces: " << count << "\n";
    if (face_count) *face_count = count;

    return shape;
}

} // namespace occ
} // namespace nl
