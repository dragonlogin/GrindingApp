#pragma once

#include <gp_Trsf.hxx>

#include "FoundationExport.h"
#include "Pose.h"
#include "JointState.h"
#include "Q.h"

namespace foundation {

Pose FOUNDATION_EXPORT ToPose(const gp_Trsf& trsf);
gp_Trsf FOUNDATION_EXPORT ToGpTrsf(const Pose& pose);

JointState FOUNDATION_EXPORT ToJointState(const nl::utils::Q& q);
nl::utils::Q FOUNDATION_EXPORT ToQ(const JointState& joints);

} // namespace foundation
