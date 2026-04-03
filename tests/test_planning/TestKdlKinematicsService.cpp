#include <QtTest>
#include "planning/KdlKinematicsService.h"
#include "foundation/Conversions.h"

class TestKdlKinematicsService : public QObject {
    Q_OBJECT

private:
    domain::Robot MakeTestRobot()
    {
        // TODO: 填入真实机器人的 source_path 和关节参数
        domain::Robot r;
        r.name = "test_robot";
        r.source_path = "";
        return r;
    }

private slots:
    // 正常值：FK 返回非单位阵的有效位姿
    void testComputeFk_returnsValidPose() { QSKIP("需配置 source_path 后启用"); }

    // Round-trip：FK(q) → IK(pose, seed=q) → FK(q2)，末端位姿误差 < 0.01mm
    void testFkIkRoundTrip() { QSKIP("需配置 source_path 后启用"); }

    // 边界值：空 joints 返回 Fail
    void testComputeFk_emptyJoints_returnsFail()
    {
        planning::KdlKinematicsService svc;
        domain::Robot robot = MakeTestRobot();
        foundation::JointState empty;
        auto result = svc.ComputeFk(robot, empty);
        // 空关节应返回 Fail 或空结果，不崩溃
        QVERIFY(!result || result.value().empty());
    }

    // 多解：ComputeIkAll 返回 ≥ 1 解
    void testComputeIkAll_multipleSolutions() { QSKIP("需配置 source_path 后启用"); }

    // 退化：超工作空间目标返回 Fail
    void testComputeIk_unreachableTarget_returnsFail() { QSKIP("需配置 source_path 后启用"); }
};

QTEST_MAIN(TestKdlKinematicsService)
#include "TestKdlKinematicsService.moc"
