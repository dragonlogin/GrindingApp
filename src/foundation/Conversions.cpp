#include "Conversions.h"

namespace foundation {

Pose ToPose(const gp_Trsf& trsf)
{
    Pose pose;

    const gp_Mat& rot = trsf.VectorialPart();
    const gp_XYZ& t = trsf.TranslationPart();

    pose.m[0] = rot.Value(1, 1);
    pose.m[1] = rot.Value(1, 2);
    pose.m[2] = rot.Value(1, 3);
    pose.m[3] = t.X();

    pose.m[4] = rot.Value(2, 1);
    pose.m[5] = rot.Value(2, 2);
    pose.m[6] = rot.Value(2, 3);
    pose.m[7] = t.Y();

    pose.m[8] = rot.Value(3, 1);
    pose.m[9] = rot.Value(3, 2);
    pose.m[10] = rot.Value(3, 3);
    pose.m[11] = t.Z();

    pose.m[12] = 0.0;
    pose.m[13] = 0.0;
    pose.m[14] = 0.0;
    pose.m[15] = 1.0;

    return pose;
}

gp_Trsf ToGpTrsf(const Pose& pose)
{
    gp_Trsf trsf;
    trsf.SetValues(
        pose.m[0], pose.m[1], pose.m[2], pose.m[3],
        pose.m[4], pose.m[5], pose.m[6], pose.m[7],
        pose.m[8], pose.m[9], pose.m[10], pose.m[11]
    );
    return trsf;
}

JointState ToJointState(const nl::utils::Q& q)
{
    JointState joints;
    joints.values_deg.resize(q.size());
    for (int i = 0; i < q.size(); ++i) {
        joints.values_deg[i] = q[i];
    }
    return joints;
}

nl::utils::Q ToQ(const JointState& joints)
{
    nl::utils::Q q(joints.size());
    for (int i = 0; i < joints.size(); ++i) {
        q[i] = joints.values_deg[i];
    }
    return q;
}

} // namespace foundation
