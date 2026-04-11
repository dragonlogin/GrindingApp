  #pragma once

  #include <string>

  #include "foundation/Result.h"
  #include "model_import/RobotDefinition.h"

  namespace model_import {

  class IRobotImporter {
  public:
      virtual ~IRobotImporter() = default;
      virtual foundation::Result<RobotDefinition> Import(const std::string& file_path) = 0;
  };

  } // namespace model_import