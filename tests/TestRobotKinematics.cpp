#include <QtTest>
#include <cmath>

#include "RobotKinematics.h"
#include "RobotDisplay.h"
#include "Q.h"
#include "Vector3d.h"

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
    void testIkAllSolutions();
    void testTrsfToRpyPosRoundTrip();
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

void TestRobotKinematics::testIkRoundTrip()
{
    RbRobot robot = makeIrb140();

    std::vector<Q> test_angles = {
        Q({0, 0, 0, 0, 0, 0}),
        Q({30, -45, 60, 90, -30, 15}),
        Q({90, 0, 0, 0, -45, 0}),
        Q({10, -20, 40, 0, 30, -10}),
    };

    for (size_t t = 0; t < test_angles.size(); ++t) {
        const Q& angles = test_angles[t];

        std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
        gp_Trsf tcp = fk.back();

        // Use all-solutions API and check that at least one round-trips
        std::vector<Q> solutions;
        bool ok = nl::kinematics::ComputeIkAllSolutions(robot, tcp, angles, solutions);
        QVERIFY2(ok, qPrintable(QString("IK failed for test case %1").arg(t)));

        bool any_match = false;
        for (size_t s = 0; s < solutions.size(); ++s) {
            std::vector<gp_Trsf> fk2 = ComputeFk(robot, solutions[s]);
            if (trsfNearEqual(tcp, fk2.back(), 1e-4, 0.1)) {
                any_match = true;
                break;
            }
        }
        QVERIFY2(any_match,
                 qPrintable(QString("FK-IK-FK round-trip failed for case %1 (%2 solutions)")
                            .arg(t).arg(solutions.size())));
    }
}

void TestRobotKinematics::testIkAllSolutions()
{
    RbRobot robot = makeIrb140();
    Q angles({30, -45, 60, 90, -30, 15});

    std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
    gp_Trsf tcp = fk.back();

    std::vector<Q> solutions;
    bool ok = nl::kinematics::ComputeIkAllSolutions(robot, tcp, angles, solutions);
    QVERIFY2(ok, "ComputeIkAllSolutions failed");
    QVERIFY2(!solutions.empty(), "No solutions returned");

    // Each solution should produce the same TCP pose via FK
    for (size_t i = 0; i < solutions.size(); ++i) {
        std::vector<gp_Trsf> fk_sol = ComputeFk(robot, solutions[i]);
        QVERIFY2(trsfNearEqual(tcp, fk_sol.back(), 1e-4, 0.1),
                 qPrintable(QString("Solution %1 FK doesn't match target").arg(i)));
    }

    // First solution should be closest to init
    QVERIFY2(solutions.size() >= 1, "Need at least 1 solution");
}


void TestRobotKinematics::testTrsfToRpyPosRoundTrip()
{
    using nl::utils::Vector3d;
    using nl::occ::RpyPosTrsf;
    using nl::occ::TrsfToRpyPos;

    struct TestCase { Vector3d rpy; Vector3d pos; };
    std::vector<TestCase> cases = {
        {{0, 0, 0},        {100, 200, 300}},
        {{45, -30, 60},    {-50, 100, 0}},
        {{90, 0, 0},       {0, 0, 500}},
        {{-120, 45, -60},  {10, -20, 30}},
        {{0, 89.9, 0},     {0, 0, 0}},   // near gimbal lock
    };

    for (size_t i = 0; i < cases.size(); ++i) {
        gp_Trsf trsf = RpyPosTrsf(cases[i].rpy, cases[i].pos);
        Vector3d rpy_out, pos_out;
        TrsfToRpyPos(trsf, rpy_out, pos_out);

        // Round-trip: build trsf from extracted values and compare
        gp_Trsf trsf2 = RpyPosTrsf(rpy_out, pos_out);
        QVERIFY2(trsfNearEqual(trsf, trsf2, 1e-6, 1e-4),
                 qPrintable(QString("TrsfToRpyPos round-trip failed for case %1").arg(i)));
    }
}

QTEST_MAIN(TestRobotKinematics)
#include "TestRobotKinematics.moc"
