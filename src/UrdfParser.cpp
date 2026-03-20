#include "UrdfParser.h"
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QXmlStreamReader>

QVector<UrdfLink> UrdfParser::Parse(const QString& urdf_path)
{
	QVector<UrdfLink> links;
	QFile file(urdf_path);
	if (!file.open(QIODevice::ReadOnly))
		return links;

	QString base_dir = QFileInfo(urdf_path).absoluteDir().absolutePath();
	QXmlStreamReader xml(&file);
	UrdfLink current_link;
	bool in_link = false;

	while (!xml.atEnd()) {
		xml.readNext();
		if (xml.isStartElement()) {
			if (xml.name() == QLatin1String("link")) {
				in_link = true;
				current_link.name = xml.attributes().value("name").toString();
				current_link.mesh_path.clear();
			}
			else if (in_link && xml.name() == QLatin1String("mesh")) {
				QString filename = xml.attributes().value("filename").toString();
				if (filename.startsWith("package://")) {
					int slash = filename.indexOf('/', 10);
					filename = (slash >= 0) ? filename.mid(slash + 1) : QString();
				}
				current_link.mesh_path = QDir(base_dir).filePath(filename);
			}
		}
		else if (xml.isEndElement()
			&& xml.name() == QLatin1String("link")) {
			links.append(current_link);
			in_link = false;
		}
	}
	return links;
}
