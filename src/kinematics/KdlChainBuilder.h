#ifndef GRINDINGAPP_SRC_KINEMATICS_KDLCHAINBUILDER_H_
#define GRINDINGAPP_SRC_KINEMATICS_KDLCHAINBUILDER_H_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include "GrindingKinematicsExport.h"
#include "RbXmlParser.h"

namespace nl {
namespace kinematics {

GRINDING_KINEMATICS_EXPORT KDL::Chain BuildKdlChain(
    const nl::core::RbRobot& robot,
    bool include_tool_mount = false);

GRINDING_KINEMATICS_EXPORT KDL::Chain BuildKdlChainFromUrdfFile(
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& tip_link);

GRINDING_KINEMATICS_EXPORT bool BuildKdlJointLimits(
    const nl::core::RbRobot& robot,
    KDL::JntArray& q_min,
    KDL::JntArray& q_max);

GRINDING_KINEMATICS_EXPORT bool BuildKdlJointLimitsFromUrdfFile(
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& tip_link,
    KDL::JntArray& q_min,
    KDL::JntArray& q_max);

} // namespace kinematics
} // namespace nl

#endif  // GRINDINGAPP_SRC_KINEMATICS_KDLCHAINBUILDER_H_
