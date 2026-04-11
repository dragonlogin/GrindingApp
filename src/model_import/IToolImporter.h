#pragma once

#include <string>

#include "foundation/Result.h"
#include "model_import/ToolDefinition.h"

namespace model_import {

class IToolImporter {
public:
    virtual ~IToolImporter() = default;
    virtual foundation::Result<ToolDefinition> Import(const std::string& file_path) = 0;
};

} // namespace model_import