#include "RbXmlParser.h"
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QDomDocument>
#include <QDebug>

RbRobot RbXmlParser::Parse(const QString& xml_path)
{
    RbRobot robot;
    QFile file(xml_path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "RbXmlParser: cannot open" << xml_path;
        return robot;
    }

    QByteArray raw = file.readAll();
    file.close();

    QDomDocument doc;
    QString err_msg; int err_line, err_col;
    if (!doc.setContent(raw, &err_msg, &err_line, &err_col)) {
        qWarning() << "RbXmlParser: XML parse error at line" << err_line
                   << "col" << err_col << ":" << err_msg;
        return robot;
    }

    QString base_dir = QFileInfo(xml_path).absoluteDir().absolutePath();
    QDomElement root = doc.documentElement();  // <SerialDevice>
    robot.name = root.attribute("name");

    // 解析 DHJoint
    QDomNodeList dh_nodes = root.elementsByTagName("DHJoint");
    for (int i = 0; i < dh_nodes.count(); ++i) {
        QDomElement el = dh_nodes.at(i).toElement();
        RbJoint j;
        j.name       = el.attribute("name");
        j.alpha_deg  = el.attribute("alpha").toDouble();
        j.a          = el.attribute("a").toDouble();
        j.d          = el.attribute("d").toDouble();
        j.offset_deg = el.attribute("offset").toDouble();
        robot.joints.append(j);
    }

    // 解析 Drawable
    QDomNodeList draw_nodes = root.elementsByTagName("Drawable");
    for (int i = 0; i < draw_nodes.count(); ++i) {
        QDomElement el = draw_nodes.at(i).toElement();
        RbDrawable d;
        d.name      = el.attribute("name");
        d.ref_joint = el.attribute("refframe");
        memset(d.rpy, 0, sizeof(d.rpy));
        memset(d.pos, 0, sizeof(d.pos));

        QDomElement rpy_el = el.firstChildElement("RPY");
        if (!rpy_el.isNull()) {
            auto parts = rpy_el.text().split(' ', Qt::SkipEmptyParts);
            if (parts.size() == 3) {
                d.rpy[0] = parts[0].toDouble();
                d.rpy[1] = parts[1].toDouble();
                d.rpy[2] = parts[2].toDouble();
            }
        }

        QDomElement pos_el = el.firstChildElement("Pos");
        if (!pos_el.isNull()) {
            auto parts = pos_el.text().split(' ', Qt::SkipEmptyParts);
            if (parts.size() == 3) {
                d.pos[0] = parts[0].toDouble();
                d.pos[1] = parts[1].toDouble();
                d.pos[2] = parts[2].toDouble();
            }
        }

        QDomElement poly_el = el.firstChildElement("Polytope");
        if (!poly_el.isNull()) {
            QString rel = poly_el.attribute("file");
            QString full = QDir(base_dir).filePath(rel + ".stl");
            if (!QFile::exists(full))
                full = QDir(base_dir).filePath(rel + ".STL");
            d.mesh_file = full;
        }

        robot.drawables.append(d);
    }

    return robot;
}
