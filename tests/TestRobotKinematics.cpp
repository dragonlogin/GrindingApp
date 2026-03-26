#include <QtTest>
#include <cmath>

#include "RobotKinematics.h"
#include "RobotDisplay.h"
#include "Q.h"

using nl::core::RbRobot;
using nl::core::RbJoint;
using nl::utils::Q;
using nl::kinematics::ComputeFk;
using nl::occ::ComputeFkHome;
using nl::occ::DhTrsf;

class TestRobotKinematics : public QObject {
    Q_OBJECT

private slots:
    void testFkMatchesManualAtHome();
    void testFkMatchesManualAtNonZero();
    void testIkRoundTrip();
};

static RbRobot makeIrb140()
{
    // IRB140 DH parameters (Craig convention, angles in deg, lengths in mm)
    RbRobot robot;
    robot.name = "IRB140";
    robot.joints.push_back({"Joint1", -90.0,  70.0, 352.0,   0.0});
    robot.joints.push_back({"Joint2",   0.0, 360.0,   0.0, -90.0});
    robot.joints.push_back({"Joint3",  90.0,   0.0,   0.0,  90.0});
    robot.joints.push_back({"Joint4", -90.0,   0.0, 380.0,   0.0});
    robot.joints.push_back({"Joint5",  90.0,   0.0,   0.0,   0.0});
    robot.joints.push_back({"Joint6",   0.0,   0.0,  65.0,   0.0});
    return robot;
}

static bool trsfNearEqual(const gp_Trsf& a, const gp_Trsf& b,
                          double rot_tol = 1e-6, double trans_tol = 1e-4)
{
    const gp_Mat& ma = a.VectorialPart();
    const gp_Mat& mb = b.VectorialPart();
    for (int r = 1; r <= 3; ++r)
        for (int c = 1; c <= 3; ++c)
            if (qAbs(ma.Value(r, c) - mb.Value(r, c)) > rot_tol) return false;

    gp_XYZ d = a.TranslationPart() - b.TranslationPart();
    return d.Modulus() <= trans_tol;
}

void TestRobotKinematics::testFkMatchesManualAtHome()
{
    RbRobot robot = makeIrb140();
    Q angles({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::vector<gp_Trsf> fk_kdl    = ComputeFk(robot, angles);
    std::vector<gp_Trsf> fk_manual = ComputeFkHome(robot);

    QCOMPARE(fk_kdl.size(), fk_manual.size());
    for (int i = 0; i < fk_kdl.size(); ++i)
        QVERIFY2(trsfNearEqual(fk_kdl[i], fk_manual[i]),
                 qPrintable(QString("Joint %1 mismatch at home").arg(i + 1)));
}

void TestRobotKinematics::testFkMatchesManualAtNonZero()
{
    RbRobot robot = makeIrb140();
    Q angles({30.0, -45.0, 60.0, 90.0, -30.0, 15.0});

    std::vector<gp_Trsf> fk_kdl = ComputeFk(robot, angles);

    // Compute reference using the manual DH loop (same logic as UpdateRobotDisplay).
    int n = robot.joints.size();
    std::vector<gp_Trsf> fk_manual(n);
    for (int i = 0; i < n; ++i) {
        const RbJoint& j = robot.joints[i];
        double theta = j.offset_deg + (i < 6 ? angles[i] : 0.0);
        gp_Trsf dh = DhTrsf(theta, j.d, j.a, j.alpha_deg);
        fk_manual[i] = (i == 0) ? dh : fk_manual[i - 1];
        if (i > 0) fk_manual[i].Multiply(dh);
    }

    QCOMPARE(fk_kdl.size(), fk_manual.size());
    for (int i = 0; i < fk_kdl.size(); ++i)
        QVERIFY2(trsfNearEqual(fk_kdl[i], fk_manual[i]),
                 qPrintable(QString("Joint %1 mismatch at non-zero angles").arg(i + 1)));
}

void TestRobotKinematics::TestRobotKinematics::testIkRoundTrip()
{
    RbRobot robot = makeIrb140();
    Q init({0, 0, 0, 0, 0, 0});

    // Test multiple poses
    std::vector<Q> test_angles = {
        Q({0, 0, 0, 0, 0, 0}),
        Q({30, -45, 60, 90, -30, 15}),
        Q({-60, 20, -30, 45, 60, -90}),
        Q({90, 0, 0, 0, -45, 0}),
    };

    for (const auto& angles : test_angles) {
        // FK
        std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
        gp_Trsf tcp = fk.back();

        // IK
        Q result(6);
        bool ok = nl::kinematics::ComputeIk(robot, tcp, angles, result);
        QVERIFY2(ok, "IK failed to converge");

        // FK again with IK result
        std::vector<gp_Trsf> fk2 = ComputeFk(robot, result);
        QVERIFY2(trsfNearEqual(fk.back(), fk2.back(), 1e-6, 0.01),
                 qPrintable(QString("FK-IK-FK round-trip failed")));
    }
}


QTEST_MAIN(TestRobotKinematics)
#include "TestRobotKinematics.moc"
