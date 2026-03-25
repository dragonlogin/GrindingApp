#include "StlLoader.h"
#include <RWStl.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <Precision.hxx>

namespace nl {
namespace occ {

TopoDS_Shape StlLoader::Load(const std::string& stl_path)
{
    Handle(Poly_Triangulation) tri = RWStl::ReadFile(stl_path.c_str());
    if (tri.IsNull())
        return TopoDS_Shape();

    BRep_Builder builder;
    TopoDS_Face face;
    builder.MakeFace(face, tri);
    return face;
}

} // namespace occ
} // namespace nl
