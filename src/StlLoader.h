#pragma once

#include <QString>
#include <TopoDS_Shape.hxx>

class StlLoader {
public:
	// ∑µªÿ Null shape ±Ì æ ß∞‹
	static TopoDS_Shape Load(const QString& stl_path);
};
