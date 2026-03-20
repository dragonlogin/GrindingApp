#pragma once

#include <QString>
#include <TopoDS_Shape.hxx>

class StlLoader {
public:
	// 返回 Null shape 表示失败
	static TopoDS_Shape Load(const QString& stl_path);
};
