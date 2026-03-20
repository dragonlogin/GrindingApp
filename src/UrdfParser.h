#pragma once
#include <QString>
#include <QVector>

struct UrdfLink {
	QString name;
	QString mesh_path;
};

class UrdfParser {
public:
	static QVector<UrdfLink> Parse(const QString& urdf_path);
};