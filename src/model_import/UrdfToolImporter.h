#pragma once

#include "ModelImportExport.h"
#include "model_import/IToolImporter.h"

namespace model_import {

class MODEL_IMPORT_EXPORT UrdfToolImporter : public IToolImporter
{
public:
    foundation::Result<ToolDefinition> Import(const std::string& file_path) override;
};

} // namespace model_import