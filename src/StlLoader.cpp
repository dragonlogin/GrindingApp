#include "StlLoader.h"
#include <RWStl.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <Precision.hxx>

TopoDS_Shape StlLoader::Load(const QString& stl_path)
{
	Handle(Poly_Triangulation) tri =
		RWStl::ReadFile(stl_path.toLocal8Bit().constData());
	if (tri.IsNull())
		return TopoDS_Shape();

	BRep_Builder builder;
	TopoDS_Face face;
	builder.MakeFace(face, tri);
	return face;
}
