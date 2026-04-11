#pragma once

#include "ModelImportExport.h"
#include "model_import/IRobotImporter.h"

namespace model_import {

class MODEL_IMPORT_EXPORT UrdfRobotImporter : public IRobotImporter
{
public:
    foundation::Result<RobotDefinition> Import(const std::string& file_path) override;
};

} // namespace model_import