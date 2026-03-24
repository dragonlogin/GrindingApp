#ifndef GRINDINGAPP_SRC_RBXMLPARSER_H_
#define GRINDINGAPP_SRC_RBXMLPARSER_H_

#include <QString>
#include <QVector>

struct RbJoint {
    QString name;
    double alpha_deg;
    double a;           // mm
    double d;           // mm
    double offset_deg;  // home position offset
};

struct RbDrawable {
    QString name;
    QString ref_joint;  // "Robot_Base" or "Joint1".."Joint6"
    double rpy[3];      // degrees: roll, pitch, yaw
    double pos[3];      // mm
    QString mesh_file;  // absolute path (with .stl extension)
};

struct RbRobot {
    QString name;
    QVector<RbJoint>    joints;
    QVector<RbDrawable> drawables;
};

class RbXmlParser {
public:
    static RbRobot Parse(const QString& xml_path);
};

#endif  // GRINDINGAPP_SRC_RBXMLPARSER_H_
