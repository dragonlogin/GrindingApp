可以，下面我直接把 `Phase 2` 拆成“可照着写”的代码骨架和落地顺序。

**Phase 2 代码骨架清单**

建议你先在 [src](/abs/path/placeholder) 下建这个目录结构：

```text
src/
  foundation/
    CMakeLists.txt
    Error.h
    Result.h
    Ids.h
    Pose.h
    JointState.h
    UnitTypes.h
    Conversions.h
    Conversions.cpp
```

你当前工作区根目录是 `E:\Code\GrindingApp`，所以上述文件实际路径建议是：

- [src/foundation/CMakeLists.txt](E:/Code/GrindingApp/src/foundation/CMakeLists.txt)
- [src/foundation/Error.h](E:/Code/GrindingApp/src/foundation/Error.h)
- [src/foundation/Result.h](E:/Code/GrindingApp/src/foundation/Result.h)
- [src/foundation/Ids.h](E:/Code/GrindingApp/src/foundation/Ids.h)
- [src/foundation/Pose.h](E:/Code/GrindingApp/src/foundation/Pose.h)
- [src/foundation/JointState.h](E:/Code/GrindingApp/src/foundation/JointState.h)
- [src/foundation/UnitTypes.h](E:/Code/GrindingApp/src/foundation/UnitTypes.h)
- [src/foundation/Conversions.h](E:/Code/GrindingApp/src/foundation/Conversions.h)
- [src/foundation/Conversions.cpp](E:/Code/GrindingApp/src/foundation/Conversions.cpp)

注意：
你还没真的创建这些文件，我这里只是在给你“应该创建什么”。

---

**1. `CMakeLists.txt` 应该怎么写**

建议先做成一个很小的共享库，后面所有新模块都依赖它。

文件：[src/foundation/CMakeLists.txt](E:/Code/GrindingApp/src/foundation/CMakeLists.txt)

```cmake
set(HEADERS
    Error.h
    Result.h
    Ids.h
    Pose.h
    JointState.h
    UnitTypes.h
    Conversions.h
)

set(SOURCES
    Conversions.cpp
)

add_library(SSFoundation SHARED ${HEADERS} ${SOURCES})

target_include_directories(SSFoundation PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(SSFoundation PUBLIC
    SSUtils
    TKernel
    TKMath
    TKG3d
)

set_target_properties(SSFoundation PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
```

说明：

- 这里先依赖 `SSUtils`，因为 `Conversions` 会桥接 `Q`
- 这里先依赖 OCCT 基础库，因为 `Conversions.cpp` 会桥接 `gp_Trsf`
- 这是“过渡期设计”，后面可以再把边界收得更紧

---

**2. `Error.h` 的建议内容**

文件：[src/foundation/Error.h](E:/Code/GrindingApp/src/foundation/Error.h)

```cpp
#pragma once

#include <string>

namespace foundation {

enum class ErrorCode {
    kOk = 0,
    kInvalidArgument,
    kNotFound,
    kAlreadyExists,
    kImportFailed,
    kGeometryFailed,
    kPlanningFailed,
    kSerializationFailed,
    kCollisionDetected,
    kStateInvalid,
    kUnsupported,
    kInternalError
};

struct Error {
    ErrorCode code = ErrorCode::kOk;
    std::string message;
    std::string detail;
};

inline Error MakeError(ErrorCode code, std::string message, std::string detail = {})
{
    return Error{code, std::move(message), std::move(detail)};
}

} // namespace foundation
```

这里的目标很简单：
- 统一错误码
- 为后面所有 service 返回值打基础

---

**3. `Result.h` 的建议内容**

文件：[src/foundation/Result.h](E:/Code/GrindingApp/src/foundation/Result.h)

```cpp
#pragma once

#include <stdexcept>
#include <utility>

#include "Error.h"

namespace foundation {

template <typename T>
class Result {
public:
    static Result Ok(T value)
    {
        return Result(std::move(value));
    }

    static Result Fail(Error error)
    {
        return Result(std::move(error));
    }

    bool ok() const { return ok_; }
    explicit operator bool() const { return ok_; }

    const T& value() const
    {
        if (!ok_) {
            throw std::logic_error("Result does not contain a value");
        }
        return value_;
    }

    T& value()
    {
        if (!ok_) {
            throw std::logic_error("Result does not contain a value");
        }
        return value_;
    }

    const Error& error() const
    {
        if (ok_) {
            throw std::logic_error("Result does not contain an error");
        }
        return error_;
    }

private:
    explicit Result(T value)
        : ok_(true), value_(std::move(value))
    {
    }

    explicit Result(Error error)
        : ok_(false), error_(std::move(error))
    {
    }

private:
    bool ok_ = false;
    T value_{};
    Error error_{};
};

template <>
class Result<void> {
public:
    static Result Ok()
    {
        return Result(true, {});
    }

    static Result Fail(Error error)
    {
        return Result(false, std::move(error));
    }

    bool ok() const { return ok_; }
    explicit operator bool() const { return ok_; }

    const Error& error() const
    {
        if (ok_) {
            throw std::logic_error("Result does not contain an error");
        }
        return error_;
    }

private:
    Result(bool ok, Error error)
        : ok_(ok), error_(std::move(error))
    {
    }

private:
    bool ok_ = false;
    Error error_{};
};

} // namespace foundation
```

这个版本先够用，不要一开始搞复杂。

---

**4. `Ids.h` 的建议内容**

文件：[src/foundation/Ids.h](E:/Code/GrindingApp/src/foundation/Ids.h)

```cpp
#pragma once

#include <string>

namespace foundation {

using ProjectId = std::string;
using SceneId = std::string;
using ObjectId = std::string;
using GeometryId = std::string;
using WaypointSetId = std::string;
using TrajectoryId = std::string;

} // namespace foundation
```

这阶段先用 `std::string` 就好。  
后面如果你想增强类型安全，再做强类型包装。

---

**5. `Pose.h` 的建议内容**

文件：[src/foundation/Pose.h](E:/Code/GrindingApp/src/foundation/Pose.h)

```cpp
#pragma once

namespace foundation {

// Row-major 4x4 homogeneous transform matrix.
// Translation unit: meters.
struct Pose {
    double m[16] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    static Pose Identity()
    {
        return Pose{};
    }
};

} // namespace foundation
```

关键点：
- 注释里把 `row-major`
- 注释里把“平移单位米”
- 以后所有跨模块位姿都以它为准

---

**6. `JointState.h` 的建议内容**

文件：[src/foundation/JointState.h](E:/Code/GrindingApp/src/foundation/JointState.h)

```cpp
#pragma once

#include <vector>

namespace foundation {

// Joint values in degrees on module boundaries.
struct JointState {
    std::vector<double> values_deg;

    int size() const
    {
        return static_cast<int>(values_deg.size());
    }

    bool empty() const
    {
        return values_deg.empty();
    }

    double operator[](int index) const
    {
        return values_deg[index];
    }

    double& operator[](int index)
    {
        return values_deg[index];
    }
};

} // namespace foundation
```

这里我建议直接加一点基础接口，不然以后每次都得写 `values_deg.size()`。

---

**7. `UnitTypes.h` 的建议内容**

文件：[src/foundation/UnitTypes.h](E:/Code/GrindingApp/src/foundation/UnitTypes.h)

```cpp
#pragma once

namespace foundation {

constexpr double kMmToM = 0.001;
constexpr double kMToMm = 1000.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

} // namespace foundation
```

这个文件虽然小，但非常重要。  
后面 UI、Planning、保存格式都要围绕它统一。

---

**8. `Conversions.h` 的建议内容**

文件：[src/foundation/Conversions.h](E:/Code/GrindingApp/src/foundation/Conversions.h)

```cpp
#pragma once

#include <gp_Trsf.hxx>

#include "Pose.h"
#include "JointState.h"
#include "Q.h"

namespace foundation {

Pose ToPose(const gp_Trsf& trsf);
gp_Trsf ToGpTrsf(const Pose& pose);

JointState ToJointState(const nl::utils::Q& q);
nl::utils::Q ToQ(const JointState& joints);

} // namespace foundation
```

注意：
这里先 include `Q.h` 和 `gp_Trsf.hxx`，是为了做过渡桥接。  
未来等迁完后，`foundation` 可以再考虑减少这类依赖。

---

**9. `Conversions.cpp` 的建议内容**

文件：[src/foundation/Conversions.cpp](E:/Code/GrindingApp/src/foundation/Conversions.cpp)

```cpp
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
```

---

**10. `src/CMakeLists.txt` 怎么接入**

你现在的 [src/CMakeLists.txt](E:/Code/GrindingApp/src/CMakeLists.txt) 是：

```cmake
include_directories(${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(utils)
add_subdirectory(core)
add_subdirectory(occ)
add_subdirectory(kinematics)
add_subdirectory(ui)
```

建议 Phase 2 改成：

```cmake
include_directories(${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(utils)
add_subdirectory(foundation)
add_subdirectory(core)
add_subdirectory(occ)
add_subdirectory(kinematics)
add_subdirectory(ui)
```

顺序上把 `foundation` 放在 `core/occ/kinematics/ui` 之前，后面新模块好依赖它。

---

**11. 顶层 `CMakeLists.txt` 这一阶段要不要改**

如果 `GrindingApp` 还没直接依赖 `SSFoundation`，顶层通常不用急着改。  
但如果你很快就会在 `ui` 或 `kinematics` 中 include foundation 头，那对应 target 要记得链上 `SSFoundation`。

也就是说，Phase 2 的最小改动通常是：

- 增加 `src/foundation`
- 在 `src/CMakeLists.txt` 里 `add_subdirectory(foundation)`

后续哪个模块用到 foundation，就在它自己的 `target_link_libraries` 里加。

---

**12. 哪些旧模块先加 `SSFoundation` 依赖**

建议先只给这两个模块加：

- [src/kinematics/CMakeLists.txt](E:/Code/GrindingApp/src/kinematics/CMakeLists.txt)
- [src/ui/CMakeLists.txt](E:/Code/GrindingApp/src/ui/CMakeLists.txt)

原因：
- 你很快会在 `TrajectoryPlanner` 和运动学桥接里用到新类型
- UI 后面也会逐步改到 `SceneSnapshot/Pose/JointState`

比如在 [src/kinematics/CMakeLists.txt](E:/Code/GrindingApp/src/kinematics/CMakeLists.txt) 里改成：

```cmake
target_link_libraries(SSKinematics PUBLIC
    SSFoundation
    GrindingCore
    Eigen3::Eigen
    orocos-kdl
    urdfdom::urdfdom_model
    urdfdom::urdfdom_world
    urdfdom::urdfdom_sensor
    urdfdom::urdfdom_model_state
    assimp::assimp
    TKernel TKMath TKG3d TKBRep
)
```

`ui` 也一样加上 `SSFoundation`。

---

**13. Phase 2 建议顺手加的测试文件**

建议新建：

- [tests/TestFoundation.cpp](E:/Code/GrindingApp/tests/TestFoundation.cpp)

内容建议覆盖：

1. `Pose` round-trip
2. `JointState` round-trip
3. `Result<int>` success/fail
4. `Result<void>` success/fail
5. 单位换算常量

一个最小测试骨架可以这样：

```cpp
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
```

---

**14. `tests/CMakeLists.txt` 也要补一下**

你现在测试 target 是偏运动学的。  
建议加一个新的测试 target，比如 `TestFoundation`。

如果你愿意保持简单，就在 [tests/CMakeLists.txt](E:/Code/GrindingApp/tests/CMakeLists.txt) 里新增一个可执行并链接：

- `SSFoundation`
- `SSUtils`
- `Qt5::Test`

---

**15. Phase 2 最推荐的实际执行顺序**

按这个顺序做，基本不会乱：

1. 创建 [src/foundation/CMakeLists.txt](E:/Code/GrindingApp/src/foundation/CMakeLists.txt)
2. 创建 [src/foundation/Error.h](E:/Code/GrindingApp/src/foundation/Error.h)
3. 创建 [src/foundation/Result.h](E:/Code/GrindingApp/src/foundation/Result.h)
4. 创建 [src/foundation/Ids.h](E:/Code/GrindingApp/src/foundation/Ids.h)
5. 创建 [src/foundation/Pose.h](E:/Code/GrindingApp/src/foundation/Pose.h)
6. 创建 [src/foundation/JointState.h](E:/Code/GrindingApp/src/foundation/JointState.h)
7. 创建 [src/foundation/UnitTypes.h](E:/Code/GrindingApp/src/foundation/UnitTypes.h)
8. 创建 [src/foundation/Conversions.h](E:/Code/GrindingApp/src/foundation/Conversions.h)
9. 创建 [src/foundation/Conversions.cpp](E:/Code/GrindingApp/src/foundation/Conversions.cpp)
10. 修改 [src/CMakeLists.txt](E:/Code/GrindingApp/src/CMakeLists.txt) 加 `add_subdirectory(foundation)`
11. 先只编译 `SSFoundation`
12. 新建 [tests/TestFoundation.cpp](E:/Code/GrindingApp/tests/TestFoundation.cpp)
13. 接入测试 target
14. 跑测试
15. 再把 `SSFoundation` 链接到 `kinematics` 和 `ui`

---

**16. Phase 2 做完后，哪些地方可以开始轻量接入**

做完后你可以马上开始在这些“新接口或过渡层”里用 foundation 类型：

- 未来的 `IKinematicsService`
- 未来的 `ITrajectoryPlanner`
- 未来的 `SceneSnapshot`
- 未来的 `SceneAppService`

但先不要急着全量改这些现有文件：

- [src/ui/MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp)
- [src/ui/RobotController.cpp](E:/Code/GrindingApp/src/ui/RobotController.cpp)
- [src/kinematics/RobotKinematics.cpp](E:/Code/GrindingApp/src/kinematics/RobotKinematics.cpp)

它们留到后续 phase，再通过桥接逐步迁。

---

**17. 这一阶段最容易踩的坑**

- `Pose` 的矩阵顺序没写清楚，后面全乱
- `JointState` 单位不统一，后面 IK/保存都会出问题
- 一上来就试图全替换 `gp_Trsf` 和 `Q`，导致改动爆炸
- `Result<T>` 设计太重，反而拖慢重构
- `foundation` 一开始就去掉所有旧依赖，结果桥接做不起来

所以这一阶段的原则就是：

- 先搭桥
- 不强拆
- 类型先统一
- 主流程先不动

如果你要，我下一条可以继续直接给你：
**Phase 2 完成后，Phase 3 的具体骨架，也就是如何把 `Waypoint/Trajectory` 真正迁到 `domain`，以及需要改哪些现有文件。**