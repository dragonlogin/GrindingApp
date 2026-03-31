#include <QtTest>

#include "Conversions.h"
#include "Result.h"
#include "UnitTypes.h"
#include <gp_Trsf.hxx>

class TestFoundation : public QObject {
    Q_OBJECT

private slots:
    void testPoseRoundTrip();
    void testJointStateRoundTrip();
    void testResultValue();
    void testResultVoid();
    void testUnits();
};

void TestFoundation::testPoseRoundTrip()
{
    gp_Trsf trsf;
    trsf.SetValues(
        1.0, 0.0, 0.0, 0.1,
        0.0, 1.0, 0.0, 0.2,
        0.0, 0.0, 1.0, 0.3
    );

    auto pose = foundation::ToPose(trsf);
    auto roundTrip = foundation::ToGpTrsf(pose);

    QCOMPARE(roundTrip.Value(1, 4), 0.1);
    QCOMPARE(roundTrip.Value(2, 4), 0.2);
    QCOMPARE(roundTrip.Value(3, 4), 0.3);
}

void TestFoundation::testJointStateRoundTrip()
{
    nl::utils::Q q({10.0, 20.0, 30.0, 40.0, 50.0, 60.0});
    auto joints = foundation::ToJointState(q);
    auto q2 = foundation::ToQ(joints);

    QCOMPARE(q2.size(), q.size());
    for (int i = 0; i < q.size(); ++i) {
        QCOMPARE(q2[i], q[i]);
    }
}

void TestFoundation::testResultValue()
{
    auto ok = foundation::Result<int>::Ok(42);
    QVERIFY(ok.ok());
    QCOMPARE(ok.value(), 42);

    auto fail = foundation::Result<int>::Fail(
        {foundation::ErrorCode::kInternalError, "boom", ""});
    QVERIFY(!fail.ok());
    QCOMPARE(fail.error().message, std::string("boom"));
}

void TestFoundation::testResultVoid()
{
    auto ok = foundation::Result<void>::Ok();
    QVERIFY(ok.ok());

    auto fail = foundation::Result<void>::Fail(
        {foundation::ErrorCode::kInvalidArgument, "bad arg", ""});
    QVERIFY(!fail.ok());
    QCOMPARE(fail.error().message, std::string("bad arg"));
}

void TestFoundation::testUnits()
{
    QCOMPARE(foundation::kMmToM * foundation::kMToMm, 1.0);
}

QTEST_MAIN(TestFoundation)
#include "TestFoundation.moc"
