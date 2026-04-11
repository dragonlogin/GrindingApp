#pragma once

#include <string>

#include "Vector3d.h"

namespace model_import {

struct ToolDefinition {
    std::string name;
    std::string source_path;
    std::string mesh_file;
    nl::utils::Vector3d mesh_scale;
    nl::utils::Vector3d base_rpy_deg;
    nl::utils::Vector3d base_pos_mm;
    nl::utils::Vector3d tcp_rpy_deg;
    nl::utils::Vector3d tcp_pos_mm;
};

} // namespace model_import