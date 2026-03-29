#include <QtTest>
#include <cmath>
#include <random>
#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>

#include <Eigen/Dense>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "RobotKinematics.h"
#include "MeshLoader.h"
#include "RobotDisplay.h"
#include "RbXmlParser.h"
#include "KdlChainBuilder.h"
#include "KdlSolver.h"
#include "Q.h"
#include "Vector3d.h"

using nl::core::RbRobot;
using nl::core::RbJoint;
using nl::utils::Q;
using nl::kinematics::ComputeFk;
using nl::kinematics::KdlSolver;

class TestRobotKinematics : public QObject {
    Q_OBJECT

private slots:
    void testFkMatchesManualAtHome();
    void testFkMatchesManualAtNonZero();
    void testDisplayHomeFkMatchesCurrentBackend();
    void testIkRoundTrip();
    void testIkAllSolutions();
    void testIkRoundTripWithParsedIrb140();
    void testSingleIkRoundTripStressWithParsedIrb140();
    void testIkRecoversSeedBranchWithParsedIrb140();
    void testSingleIkRandomStressWithParsedIrb140();
    void testParsedIrb140ReturnsMultipleIkSolutions();
    void testParsedIrb140AllSolutionsMatchTargetRandom();
    void testParsedIrb140SolutionsAreUniqueAtWristSingularity();
    void testKdlSolverMatchesManualAtHome();
    void testKdlSolverIkRoundTripWithParsedIrb140();
    void testBuildKdlChainFromUrdfdomForIrb2400();
    void testBuildKdlChainWrapperMatchesDirectUrdfChainForIrb2400();
    void testParsedIrb2400UrdfLoadsRobotModel();
    void testMeshLoaderLoadsIrb2400StlMesh();
    void testTrsfToRpyPosRoundTrip();
};

static RbRobot makeIrb140()
{
    const QString tests_dir = QFileInfo(QStringLiteral(__FILE__)).absolutePath();
    const QString urdf_path = QDir(tests_dir).absoluteFilePath("../model/robot/IRB140/urdf/IRB140.urdf");
    return nl::core::RbXmlParser::Parse(QDir::cleanPath(urdf_path).toStdString());
}

static QString makeIrb2400UrdfPath()
{
    const QString tests_dir = QFileInfo(QStringLiteral(__FILE__)).absolutePath();
    return QDir(tests_dir).absoluteFilePath("../model/robot/irb2400/irb2400.urdf");
}

static RbRobot makeIrb2400()
{
    return nl::core::RbXmlParser::Parse(QDir::cleanPath(makeIrb2400UrdfPath()).toStdString());
}

static QString makeIrb140UrdfPath(const RbRobot& robot)
{
    if (robot.source_path.empty()) return {};

    const QString source_path = QString::fromStdString(robot.source_path);
    if (source_path.endsWith(".urdf", Qt::CaseInsensitive))
        return QDir::cleanPath(source_path);

    const QFileInfo xml_info(source_path);
    return QDir::cleanPath(
        xml_info.absoluteDir().filePath(QStringLiteral("urdf/%1.urdf")
                                        .arg(QString::fromStdString(robot.name))));
}

static gp_Trsf eigenToGpTrsf(const Eigen::Matrix4d& m)
{
    gp_Trsf t;
    t.SetValues(
        m(0, 0), m(0, 1), m(0, 2), m(0, 3),
        m(1, 0), m(1, 1), m(1, 2), m(1, 3),
        m(2, 0), m(2, 1), m(2, 2), m(2, 3));
    return t;
}

static Eigen::Vector3d parseUrdfTriple(const QString& text, const Eigen::Vector3d& fallback)
{
    const auto parts = text.split(' ', Qt::SkipEmptyParts);
    if (parts.size() != 3) return fallback;

    return Eigen::Vector3d(parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble());
}

static Eigen::Vector3d parseUrdfRpyDegreesAsRadians(const QString& text)
{
    // Robot URDF files in this repo store RPY values in degrees for authoring.
    // Runtime KDL converts them back to radians before handing them to urdfdom,
    // and the reference FK in tests needs the same conversion.
    const Eigen::Vector3d deg = parseUrdfTriple(text, Eigen::Vector3d::Zero());
    return deg * M_PI / 180.0;
}

static Eigen::Matrix4d makeUrdfOriginMatrix(const Eigen::Vector3d& xyz,
                                            const Eigen::Vector3d& rpy_rad)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(rpy_rad.z(), Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(rpy_rad.y(), Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(rpy_rad.x(), Eigen::Vector3d::UnitX()).toRotationMatrix();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = xyz;
    return transform;
}

static std::vector<gp_Trsf> computeUrdfReferenceFk(const RbRobot& robot, const Q& angles)
{
    std::vector<gp_Trsf> frames;
    const QString urdf_path = makeIrb140UrdfPath(robot);
    if (urdf_path.isEmpty()) return frames;

    QFile file(urdf_path);
    if (!file.open(QIODevice::ReadOnly)) return frames;

    QDomDocument doc;
    QString err_msg;
    int err_line = 0;
    int err_col = 0;
    if (!doc.setContent(file.readAll(), &err_msg, &err_line, &err_col))
        return frames;

    Eigen::Matrix4d current = Eigen::Matrix4d::Identity();
    const QDomNodeList joint_nodes = doc.documentElement().elementsByTagName("joint");
    int angle_index = 0;
    for (int i = 0; i < joint_nodes.count(); ++i) {
        const QDomElement joint_el = joint_nodes.at(i).toElement();
        const QString joint_type = joint_el.attribute("type");

        const QDomElement origin_el = joint_el.firstChildElement("origin");
        const Eigen::Vector3d xyz =
            parseUrdfTriple(origin_el.attribute("xyz"), Eigen::Vector3d::Zero());
        const Eigen::Vector3d rpy =
            parseUrdfRpyDegreesAsRadians(origin_el.attribute("rpy"));
        current *= makeUrdfOriginMatrix(xyz, rpy);

        if (joint_type == QStringLiteral("fixed"))
            continue;

        const QDomElement axis_el = joint_el.firstChildElement("axis");
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        if (!axis_el.isNull())
            axis = parseUrdfTriple(axis_el.attribute("xyz"), Eigen::Vector3d::UnitZ());
        if (axis.norm() < 1e-9)
            axis = Eigen::Vector3d::UnitZ();
        axis.normalize();

        const double angle_deg = angle_index < angles.size() ? angles[angle_index] : 0.0;
        const double offset_deg =
            angle_index < static_cast<int>(robot.joints.size()) ? robot.joints[angle_index].offset_deg : 0.0;
        const double angle_rad = qDegreesToRadians(angle_deg + offset_deg);
        const Eigen::Matrix3d joint_rotation =
            Eigen::AngleAxisd(angle_rad, axis).toRotationMatrix();
        current.block<3, 3>(0, 0) = current.block<3, 3>(0, 0) * joint_rotation;

        frames.push_back(eigenToGpTrsf(current));
        ++angle_index;
    }

    return frames;
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

static double normalizeDeg(double deg)
{
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

static bool jointNearEqual(const Q& a, const Q& b, double tol_deg = 1e-3)
{
    if (a.size() != b.size()) return false;
    for (int i = 0; i < a.size(); ++i) {
        if (std::abs(normalizeDeg(a[i] - b[i])) > tol_deg) return false;
    }
    return true;
}

void TestRobotKinematics::testFkMatchesManualAtHome()
{
    RbRobot robot = makeIrb140();
    Q angles({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::vector<gp_Trsf> fk_kdl    = ComputeFk(robot, angles);
    std::vector<gp_Trsf> fk_manual = computeUrdfReferenceFk(robot, angles);

    QCOMPARE(fk_kdl.size(), fk_manual.size());
    for (int i = 0; i < fk_kdl.size(); ++i)
        QVERIFY2(trsfNearEqual(fk_kdl[i], fk_manual[i]),
                 qPrintable(QString("Joint %1 mismatch against URDF at home").arg(i + 1)));
}

void TestRobotKinematics::testFkMatchesManualAtNonZero()
{
    RbRobot robot = makeIrb140();
    Q angles({30.0, -45.0, 60.0, 90.0, -30.0, 15.0});

    std::vector<gp_Trsf> fk_kdl    = ComputeFk(robot, angles);
    std::vector<gp_Trsf> fk_manual = computeUrdfReferenceFk(robot, angles);

    QCOMPARE(fk_kdl.size(), fk_manual.size());
    for (int i = 0; i < fk_kdl.size(); ++i)
        QVERIFY2(trsfNearEqual(fk_kdl[i], fk_manual[i]),
                 qPrintable(QString("Joint %1 mismatch against URDF at non-zero angles").arg(i + 1)));
}

void TestRobotKinematics::testDisplayHomeFkMatchesCurrentBackend()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    Q home_angles({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    std::vector<gp_Trsf> fk_runtime = ComputeFk(robot, home_angles);
    std::vector<gp_Trsf> fk_display = nl::occ::ComputeFkHome(robot);

    QCOMPARE(fk_runtime.size(), fk_display.size());
    for (int i = 0; i < fk_runtime.size(); ++i) {
        QVERIFY2(trsfNearEqual(fk_runtime[i], fk_display[i]),
                 qPrintable(QString("Display home FK mismatch at joint %1").arg(i + 1)));
    }
}

void TestRobotKinematics::testIkRoundTrip()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    // Keep cases within the real IRB140 model's joint limits and away from
    // wrist singularities so FK->IK->FK remains a stable regression check.
    std::vector<Q> test_angles = {
        Q({0, 0, 0, 0, 0, 0}),
        Q({20, -30, -40, 50, 10, -15}),
        Q({-45, 15, -80, 90, -20, 35}),
        Q({10, -20, -30, 40, 15, -10}),
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
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");
    Q angles({20, -40, -60, 80, -20, 30});

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

void TestRobotKinematics::testIkRoundTripWithParsedIrb140()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    std::vector<Q> test_angles = {
        Q({0, 0, 0, 0, 0, 0}),
        Q({20, -30, -40, 50, 10, -15}),
        Q({-45, 15, -80, 90, -20, 35}),
    };

    for (size_t t = 0; t < test_angles.size(); ++t) {
        const Q& angles = test_angles[t];
        std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
        QVERIFY2(!fk.empty(), qPrintable(QString("FK returned empty for case %1").arg(t)));

        gp_Trsf flange = fk.back();
        std::vector<Q> solutions;
        bool ok = nl::kinematics::ComputeIkAllSolutions(robot, flange, angles, solutions);
        QVERIFY2(ok, qPrintable(QString("IK failed for parsed IRB140 case %1").arg(t)));
        QVERIFY2(!solutions.empty(),
                 qPrintable(QString("No IK solutions for parsed IRB140 case %1").arg(t)));

        bool any_match = false;
        for (size_t s = 0; s < solutions.size(); ++s) {
            std::vector<gp_Trsf> fk2 = ComputeFk(robot, solutions[s]);
            if (trsfNearEqual(flange, fk2.back(), 1e-4, 0.1)) {
                any_match = true;
                break;
            }
        }

        QVERIFY2(any_match,
                 qPrintable(QString("Parsed IRB140 FK-IK-FK round-trip failed for case %1 (%2 solutions)")
                            .arg(t).arg(solutions.size())));
    }
}

void TestRobotKinematics::testSingleIkRoundTripStressWithParsedIrb140()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    std::vector<Q> test_angles = {
        Q({0, 0, 0, 0, 0, 0}),
        Q({10, -20, -30, 40, 15, -10}),
        Q({20, -40, -60, 80, -20, 30}),
        Q({-30, 25, -90, 60, 35, -45}),
        Q({45, -10, -110, 120, -40, 60}),
        Q({-60, 35, -70, -90, 50, -75}),
    };

    for (size_t t = 0; t < test_angles.size(); ++t) {
        const Q& angles = test_angles[t];
        std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
        QVERIFY2(!fk.empty(), qPrintable(QString("FK returned empty for stress case %1").arg(t)));

        gp_Trsf flange = fk.back();
        Q ik_result(6, 0.0);
        bool ok = nl::kinematics::ComputeIk(robot, flange, angles, ik_result);
        QVERIFY2(ok, qPrintable(QString("Single IK failed for stress case %1").arg(t)));

        std::vector<gp_Trsf> fk2 = ComputeFk(robot, ik_result);
        QVERIFY2(trsfNearEqual(flange, fk2.back(), 1e-4, 0.1),
                 qPrintable(QString("Single IK FK-IK-FK round-trip failed for stress case %1").arg(t)));
    }
}

void TestRobotKinematics::testIkRecoversSeedBranchWithParsedIrb140()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    std::vector<Q> test_angles = {
        Q({10, -20, -30, 40, 15, -10}),
        Q({20, -40, -60, 80, -20, 30}),
        Q({-30, 25, -90, 60, 35, -45}),
        Q({45, -10, -110, 120, -40, 60}),
    };

    for (size_t t = 0; t < test_angles.size(); ++t) {
        const Q& angles = test_angles[t];
        std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
        QVERIFY2(!fk.empty(), qPrintable(QString("FK returned empty for branch case %1").arg(t)));

        std::vector<Q> solutions;
        bool ok = nl::kinematics::ComputeIkAllSolutions(robot, fk.back(), angles, solutions);
        QVERIFY2(ok, qPrintable(QString("IK failed for branch case %1").arg(t)));
        QVERIFY2(!solutions.empty(),
                 qPrintable(QString("No solutions returned for branch case %1").arg(t)));

        QVERIFY2(jointNearEqual(solutions.front(), angles, 1e-2),
                 qPrintable(QString("Closest IK branch did not recover seed for case %1").arg(t)));
    }
}

void TestRobotKinematics::testSingleIkRandomStressWithParsedIrb140()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    std::mt19937 rng(140);
    std::uniform_real_distribution<double> j1(-170.0, 170.0);
    std::uniform_real_distribution<double> j2(-70.0, 100.0);
    std::uniform_real_distribution<double> j3(-180.0, 20.0);
    std::uniform_real_distribution<double> j4(-170.0, 170.0);
    std::uniform_real_distribution<double> j5(-100.0, 100.0);
    std::uniform_real_distribution<double> j6(-180.0, 180.0);

    for (int t = 0; t < 200; ++t) {
        Q angles({j1(rng), j2(rng), j3(rng), j4(rng), j5(rng), j6(rng)});
        std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
        QVERIFY2(!fk.empty(), qPrintable(QString("FK returned empty for random case %1").arg(t)));

        Q ik_result(6, 0.0);
        bool ok = nl::kinematics::ComputeIk(robot, fk.back(), angles, ik_result);
        QVERIFY2(ok, qPrintable(QString("Single IK failed for random case %1").arg(t)));

        std::vector<gp_Trsf> fk2 = ComputeFk(robot, ik_result);
        QVERIFY2(trsfNearEqual(fk.back(), fk2.back(), 1e-4, 0.1),
                 qPrintable(QString("Single IK FK-IK-FK round-trip failed for random case %1").arg(t)));
    }
}

void TestRobotKinematics::testParsedIrb140ReturnsMultipleIkSolutions()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    Q angles({20, -40, -60, 80, -20, 30});
    std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
    QVERIFY2(!fk.empty(), "FK returned empty for parsed IRB140");

    std::vector<Q> solutions;
    bool ok = nl::kinematics::ComputeIkAllSolutions(robot, fk.back(), angles, solutions);
    QVERIFY2(ok, "ComputeIkAllSolutions failed for parsed IRB140");
    QVERIFY2(solutions.size() > 1,
             qPrintable(QString("Expected analytical multi-solution IK for parsed IRB140, got %1 solution(s)")
                        .arg(solutions.size())));
}

void TestRobotKinematics::testParsedIrb140AllSolutionsMatchTargetRandom()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    std::mt19937 rng(280);
    std::uniform_real_distribution<double> j1(-170.0, 170.0);
    std::uniform_real_distribution<double> j2(-70.0, 100.0);
    std::uniform_real_distribution<double> j3(-180.0, 20.0);
    std::uniform_real_distribution<double> j4(-170.0, 170.0);
    std::uniform_real_distribution<double> j5(-100.0, 100.0);
    std::uniform_real_distribution<double> j6(-180.0, 180.0);

    for (int t = 0; t < 60; ++t) {
        Q angles({j1(rng), j2(rng), j3(rng), j4(rng), j5(rng), j6(rng)});
        std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
        QVERIFY2(!fk.empty(), qPrintable(QString("FK returned empty for all-solutions case %1").arg(t)));

        std::vector<Q> solutions;
        bool ok = nl::kinematics::ComputeIkAllSolutions(robot, fk.back(), angles, solutions);
        QVERIFY2(ok, qPrintable(QString("IK failed for all-solutions case %1").arg(t)));
        QVERIFY2(!solutions.empty(), qPrintable(QString("No solutions for all-solutions case %1").arg(t)));

        for (size_t i = 0; i < solutions.size(); ++i) {
            std::vector<gp_Trsf> fk_sol = ComputeFk(robot, solutions[i]);
            QVERIFY2(trsfNearEqual(fk.back(), fk_sol.back(), 1e-4, 0.1),
                     qPrintable(QString("Random all-solutions case %1 solution %2 FK mismatch")
                                .arg(t).arg(i)));
        }
    }
}

void TestRobotKinematics::testParsedIrb140SolutionsAreUniqueAtWristSingularity()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    Q angles({0, 0, 0, 0, 0, 0});
    std::vector<gp_Trsf> fk = ComputeFk(robot, angles);
    QVERIFY2(!fk.empty(), "FK returned empty for singularity test");

    std::vector<Q> solutions;
    bool ok = nl::kinematics::ComputeIkAllSolutions(robot, fk.back(), angles, solutions);
    QVERIFY2(ok, "IK failed for singularity test");
    QVERIFY2(!solutions.empty(), "No solutions for singularity test");

    for (size_t i = 0; i < solutions.size(); ++i) {
        for (size_t j = i + 1; j < solutions.size(); ++j) {
            QVERIFY2(!jointNearEqual(solutions[i], solutions[j], 1e-2),
                     qPrintable(QString("Duplicate IK solutions at singularity: %1 and %2")
                                .arg(i).arg(j)));
        }
    }
}

void TestRobotKinematics::testKdlSolverMatchesManualAtHome()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    KdlSolver solver;
    Q angles({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    std::vector<gp_Trsf> fk_kdl = solver.ComputeFk(robot, angles);
    std::vector<gp_Trsf> fk_manual = computeUrdfReferenceFk(robot, angles);

    QCOMPARE(fk_kdl.size(), fk_manual.size());
    for (int i = 0; i < fk_kdl.size(); ++i) {
        QVERIFY2(trsfNearEqual(fk_kdl[i], fk_manual[i]),
                 qPrintable(QString("KDL solver FK mismatch against URDF at home joint %1").arg(i + 1)));
    }
}

void TestRobotKinematics::testKdlSolverIkRoundTripWithParsedIrb140()
{
    RbRobot robot = makeIrb140();
    QVERIFY2(!robot.joints.empty(), "Failed to load IRB140.urdf");

    KdlSolver solver;
    Q angles({20, -40, -60, 80, -20, 30});

    std::vector<gp_Trsf> fk = solver.ComputeFk(robot, angles);
    QVERIFY2(!fk.empty(), "KDL solver FK returned empty");

    Q ik_result(6, 0.0);
    bool ok = solver.ComputeIk(robot, fk.back(), angles, ik_result);
    QVERIFY2(ok, "KDL solver IK failed");

    std::vector<gp_Trsf> fk2 = solver.ComputeFk(robot, ik_result);
    QVERIFY2(trsfNearEqual(fk.back(), fk2.back(), 1e-4, 0.1),
             "KDL solver FK-IK-FK round-trip failed");
}

void TestRobotKinematics::testBuildKdlChainFromUrdfdomForIrb2400()
{
    const std::string urdf_path = QDir::cleanPath(makeIrb2400UrdfPath()).toStdString();
    KDL::Chain chain = nl::kinematics::BuildKdlChainFromUrdfFile(
        urdf_path, "base_link", "tool0");

    QCOMPARE(chain.getNrOfJoints(), 6u);
    QCOMPARE(chain.getNrOfSegments(), 7u);

    KDL::JntArray q_min;
    KDL::JntArray q_max;
    QVERIFY2(nl::kinematics::BuildKdlJointLimitsFromUrdfFile(
                 urdf_path, "base_link", "tool0", q_min, q_max),
             "Failed to read IRB2400 joint limits from urdfdom");
    QCOMPARE(q_min.rows(), 6u);
    QCOMPARE(q_max.rows(), 6u);
    QVERIFY(std::abs(q_min(1) + 1.7453) < 1e-4);
    QVERIFY(std::abs(q_max(1) - 1.9199) < 1e-4);

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::JntArray q_zero(chain.getNrOfJoints());
    KDL::Frame tip;
    QVERIFY2(fk_solver.JntToCart(q_zero, tip) >= 0,
             "urdfdom-built KDL chain failed FK for IRB2400");
}

void TestRobotKinematics::testBuildKdlChainWrapperMatchesDirectUrdfChainForIrb2400()
{
    RbRobot robot = makeIrb2400();
    QVERIFY2(!robot.joints.empty(), "Failed to load irb2400.urdf");

    KDL::Chain wrapper_chain = nl::kinematics::BuildKdlChain(robot, false);
    KDL::Chain direct_chain = nl::kinematics::BuildKdlChainFromUrdfFile(
        QDir::cleanPath(makeIrb2400UrdfPath()).toStdString(), "base_link", "link_6");

    QCOMPARE(wrapper_chain.getNrOfJoints(), direct_chain.getNrOfJoints());
    QCOMPARE(wrapper_chain.getNrOfSegments(), direct_chain.getNrOfSegments());
    QCOMPARE(wrapper_chain.getNrOfJoints(), 6u);
    QCOMPARE(wrapper_chain.getNrOfSegments(), 6u);
}

void TestRobotKinematics::testParsedIrb2400UrdfLoadsRobotModel()
{
    RbRobot robot = makeIrb2400();
    QCOMPARE(QString::fromStdString(robot.name), QStringLiteral("abb_irb2400"));
    QCOMPARE(robot.joints.size(), 6u);
    QVERIFY2(robot.drawables.size() >= 7, "IRB2400 URDF should expose base + 6 display meshes");
    QVERIFY(QFileInfo(QString::fromStdString(robot.drawables.front().mesh_file)).exists());
    QVERIFY2(QString::fromStdString(robot.drawables.front().mesh_file).endsWith(".stl", Qt::CaseInsensitive),
             "IRB2400 import should currently prefer STL meshes for display");
}

void TestRobotKinematics::testMeshLoaderLoadsIrb2400StlMesh()
{
    RbRobot robot = makeIrb2400();
    QVERIFY2(!robot.drawables.empty(), "IRB2400 URDF returned no drawables");

    TopoDS_Shape shape = nl::occ::MeshLoader::Load(robot.drawables.front().mesh_file);
    QVERIFY2(!shape.IsNull(), "MeshLoader failed to load IRB2400 STL mesh");
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
