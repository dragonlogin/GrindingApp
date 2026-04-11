#include "UrdfRobotImporter.h"

#include "RbXmlParser.h"
#include "domain/Robot.h"
#include "foundation/Error.h"

namespace model_import {

namespace {

RobotDefinition ToRobotDefinition(const nl::core::RbRobot& rb)
{
    RobotDefinition def;
    def.model.name = rb.name;
    def.model.source_path = rb.source_path;
    for (const auto& j : rb.joints) {
        domain::RobotJoint dj;
        dj.name = j.name;
        dj.alpha_deg = j.alpha_deg;
        dj.a_mm = j.a;
        dj.d_mm = j.d;
        dj.offset_deg = j.offset_deg;
        def.model.joints.push_back(dj);
    }
    for (const auto& drw : rb.drawables) {
        MeshRef mesh;
        mesh.name = drw.name;
        mesh.ref_joint = drw.ref_joint;
        mesh.rpy = drw.rpy;
        mesh.pos = drw.pos;
        mesh.mesh_file = drw.mesh_file;
        def.meshes.push_back(mesh);
    }
    return def;
}

} // namespace

foundation::Result<RobotDefinition>
UrdfRobotImporter::Import(const std::string& file_path)
{
    nl::core::RbRobot rb = nl::core::RbXmlParser::Parse(file_path);
    if (rb.name.empty() || rb.joints.empty())
        return foundation::Result<RobotDefinition>::Fail(
            foundation::Error{foundation::ErrorCode::kImportFailed, "Failed to parse robot file: " + file_path, ""});
    return foundation::Result<RobotDefinition>::Ok(ToRobotDefinition(rb));
}

} // namespace model_import