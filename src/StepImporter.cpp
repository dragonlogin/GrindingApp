#include "StepImporter.h"

#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <iostream>

TopoDS_Shape StepImporter::Load(const QString& file_path, int* face_count)
{
	STEPControl_Reader reader;
	IFSelect_ReturnStatus status =
		reader.ReadFile(file_path.toStdString().c_str());

	if (status != IFSelect_RetDone) {
		std::cerr << "[StepImporter] Failed to read: "
			<< file_path.toStdString() << "\n";
		return TopoDS_Shape();
	}

	reader.TransferRoots();
	TopoDS_Shape shape = reader.OneShape();

	// 三角化（线性偏差 0.1 mm）
	BRepMesh_IncrementalMesh(shape, 0.1);

	// 统计面数
	int count = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
		++count;

	std::cout << "[StepImporter] Faces: " << count << "\n";
	if (face_count) *face_count = count;

	return shape;
}
