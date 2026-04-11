  #pragma once

  #include <string>
  #include <vector>

  #include "domain/Robot.h"
  #include "Vector3d.h"

  namespace model_import {

  struct MeshRef {
      std::string name;
      std::string ref_joint;        // "Robot_Base" 或 "Joint1".."Joint6"
      nl::utils::Vector3d rpy;      // degrees: yaw, pitch, roll
      nl::utils::Vector3d pos;      // mm
      std::string mesh_file;        // absolute path
  };

  struct RobotDefinition {
      domain::Robot model;
      std::vector<MeshRef> meshes;
  };

  } // namespace model_import