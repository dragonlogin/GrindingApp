#pragma once

#include <QString>
#include <TopoDS_Shape.hxx>

class StepImporter
{
public:
	// 读取 STEP 文件并三角化，返回形状；faceCount 可为 nullptr
	static TopoDS_Shape Load(const QString& file_path, int* face_count = nullptr);
};
