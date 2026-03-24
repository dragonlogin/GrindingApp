#include <QtTest>
#include <cmath>

#include "RobotKinematics.h"
#include "RobotDisplay.h"

class TestRobotKinematics : public QObject {
    Q_OBJECT

private slots:
    void testFkMatchesManualAtHome();
    void testFkMatchesManualAtNonZero();
};

static RbRobot makeIrb140()
{
    // IRB140 DH parameters (Craig convention, angles in deg, lengths in mm)
    RbRobot robot;
    robot.name = "IRB140";
    robot.joints.append({"Joint1", -90.0,  70.0, 352.0,   0.0});
    robot.joints.append({"Joint2",   0.0, 360.0,   0.0, -90.0});
    robot.joints.append({"Joint3",  90.0,   0.0,   0.0,  90.0});
    robot.joints.append({"Joint4", -90.0,   0.0, 380.0,   0.0});
    robot.joints.append({"Joint5",  90.0,   0.0,   0.0,   0.0});
    robot.joints.append({"Joint6",   0.0,   0.0,  65.0,   0.0});
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
    double angles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    QVector<gp_Trsf> fk_kdl    = ComputeFkKdl(robot, angles);
    QVector<gp_Trsf> fk_manual = ComputeFkHome(robot);

    QCOMPARE(fk_kdl.size(), fk_manual.size());
    for (int i = 0; i < fk_kdl.size(); ++i)
        QVERIFY2(trsfNearEqual(fk_kdl[i], fk_manual[i]),
                 qPrintable(QString("Joint %1 mismatch at home").arg(i + 1)));
}

void TestRobotKinematics::testFkMatchesManualAtNonZero()
{
    RbRobot robot = makeIrb140();
    double angles[6] = {30.0, -45.0, 60.0, 90.0, -30.0, 15.0};

    QVector<gp_Trsf> fk_kdl = ComputeFkKdl(robot, angles);

    // Compute reference using the manual DH loop (same logic as UpdateRobotDisplay).
    int n = robot.joints.size();
    QVector<gp_Trsf> fk_manual(n);
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

QTEST_MAIN(TestRobotKinematics)
#include "TestRobotKinematics.moc"
