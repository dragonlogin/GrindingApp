下面直接给你 `Phase 3` 的可执行实现方案。这个阶段的核心只有一句话：

把“路径点 / 轨迹”从 `occ` 命名空间和模块里剥离出来，变成真正的领域对象，但先通过兼容层保证现有功能不炸。

**Phase 3 目标**

完成这几件事：

- `Waypoint` 不再定义为 `occ` 的概念
- `Trajectory` 不再定义为 `occ` 的概念
- `TrajectoryPlanner`、`MainWindow`、后续 `Application` 都开始面向 `domain` 类型编程
- 现有 UI 功能不回归
- 代码仍然可编译、可运行、可验证

这一阶段先不强行把所有逻辑都迁走，但要把“数据模型归属”先纠正。

---

**Phase 3 最终产出**

建议新增这些文件：

- [src/domain/CMakeLists.txt](E:/Code/GrindingApp/src/domain/CMakeLists.txt)
- [src/domain/Waypoint.h](E:/Code/GrindingApp/src/domain/Waypoint.h)
- [src/domain/WaypointSet.h](E:/Code/GrindingApp/src/domain/WaypointSet.h)
- [src/domain/Trajectory.h](E:/Code/GrindingApp/src/domain/Trajectory.h)

如果你想更稳一点，也可以加：

- [src/domain/PlanningTypes.h](E:/Code/GrindingApp/src/domain/PlanningTypes.h)

然后逐步把这些旧文件从“定义者”改成“兼容转发者”：

- [src/occ/Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h)
- [src/occ/Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h)

---

## 3.1 Domain 模块先建起来

如果 `Phase 1` 你还没真建 `domain` target，这一阶段必须补上。

### `src/domain/CMakeLists.txt`

```cmake
set(HEADERS
    Waypoint.h
    WaypointSet.h
    Trajectory.h
)

set(SOURCES
)

add_library(SSDomain SHARED ${HEADERS} ${SOURCES})

target_include_directories(SSDomain PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(SSDomain PUBLIC
    SSFoundation
)

set_target_properties(SSDomain PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
```

### `src/CMakeLists.txt` 增加

在 [src/CMakeLists.txt](E:/Code/GrindingApp/src/CMakeLists.txt) 里加：

```cmake
add_subdirectory(domain)
```

建议顺序：

```cmake
include_directories(${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(utils)
add_subdirectory(foundation)
add_subdirectory(domain)
add_subdirectory(core)
add_subdirectory(occ)
add_subdirectory(kinematics)
add_subdirectory(ui)
```

---

## 3.2 新的 Domain `Waypoint` 怎么定义

文件：[src/domain/Waypoint.h](E:/Code/GrindingApp/src/domain/Waypoint.h)

```cpp
#pragma once

#include "Pose.h"

namespace domain {

struct Waypoint {
    foundation::Pose pose;
    double speed_ratio = 1.0;
};

} // namespace domain
```

这里不要再 include `gp_Trsf.hxx`。  
如果你还在过渡期需要 OCCT，桥接应该放在 geometry/planning 或 conversion 层，不放在 domain。

---

## 3.3 新的 `WaypointSet` 怎么定义

文件：[src/domain/WaypointSet.h](E:/Code/GrindingApp/src/domain/WaypointSet.h)

```cpp
#pragma once

#include <string>
#include <vector>

#include "Ids.h"
#include "Waypoint.h"

namespace domain {

struct WaypointSet {
    foundation::WaypointSetId id;
    std::vector<Waypoint> points;
    std::string source_face_token;
};

} // namespace domain
```

这阶段先最小定义就行。  
`source_face_token` 先用 `std::string` 过渡，后面再换成正式 `FaceRef`。

---

## 3.4 新的 Domain `Trajectory` 怎么定义

文件：[src/domain/Trajectory.h](E:/Code/GrindingApp/src/domain/Trajectory.h)

```cpp
#pragma once

#include <vector>

#include "Ids.h"
#include "Pose.h"
#include "JointState.h"

namespace domain {

enum class MoveType {
    kMoveJ,
    kMoveL
};

enum class TrajectoryPointStatus {
    kOk,
    kIkFailed,
    kJointJump,
    kCollision
};

struct TrajectoryPoint {
    foundation::Pose tcp_pose;
    foundation::JointState joint_state;
    MoveType move_type = MoveType::kMoveL;
    TrajectoryPointStatus status = TrajectoryPointStatus::kOk;
    int waypoint_index = -1;
};

struct Trajectory {
    foundation::TrajectoryId id;
    std::vector<TrajectoryPoint> points;

    bool HasErrors() const
    {
        for (const auto& p : points) {
            if (p.status != TrajectoryPointStatus::kOk) {
                return true;
            }
        }
        return false;
    }

    int ErrorCount() const
    {
        int count = 0;
        for (const auto& p : points) {
            if (p.status != TrajectoryPointStatus::kOk) {
                ++count;
            }
        }
        return count;
    }
};

} // namespace domain
```

这里最关键的是：

- `TrajectoryPoint::joint_angles` 改成 `JointState`
- `gp_Trsf` 改成 `Pose`
- `occ` 的业务归属被拿掉

---

## 3.5 旧 `occ` 头文件怎么做兼容层

这一步很关键，因为它能让你不需要一次性改完整个仓库。

### 改 [src/occ/Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h)

建议临时改成：

```cpp
#pragma once

#include "domain/Waypoint.h"

namespace nl {
namespace occ {

using Waypoint = domain::Waypoint;

} // namespace occ
} // namespace nl
```

### 改 [src/occ/Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h)

建议临时改成：

```cpp
#pragma once

#include "domain/Trajectory.h"

namespace nl {
namespace occ {

using Trajectory = domain::Trajectory;
using TrajectoryPoint = domain::TrajectoryPoint;

} // namespace occ
} // namespace nl
```

如果编译时枚举名不兼容，还可以继续补：

```cpp
using MoveType = domain::MoveType;
using TrajectoryPointStatus = domain::TrajectoryPointStatus;
```

这样做的好处：

- 旧代码大部分还能编译
- 新代码已经开始走 `domain`
- 你可以逐文件迁，不用一次性大爆炸

---

## 3.6 需要先补的转换函数

因为老代码还在用 `gp_Trsf` 和 `Q`，而新 domain 类型已经用 `Pose` 和 `JointState`，所以 Phase 3 要把桥接函数补齐。

建议在 [src/foundation/Conversions.h](E:/Code/GrindingApp/src/foundation/Conversions.h) 里保留这些函数：

```cpp
Pose ToPose(const gp_Trsf& trsf);
gp_Trsf ToGpTrsf(const Pose& pose);

JointState ToJointState(const nl::utils::Q& q);
nl::utils::Q ToQ(const JointState& joints);
```

并且新增两个轻量辅助：

```cpp
namespace foundation {

inline domain::Waypoint ToWaypoint(const gp_Trsf& pose, double speed_ratio = 1.0)
{
    return domain::Waypoint{ToPose(pose), speed_ratio};
}

inline gp_Trsf ToGpTrsf(const domain::Waypoint& waypoint)
{
    return ToGpTrsf(waypoint.pose);
}

} // namespace foundation
```

这个可以放在单独头里，比如后面新增：

- [src/foundation/DomainConversions.h](E:/Code/GrindingApp/src/foundation/DomainConversions.h)

但 Phase 3 不一定非要拆。

---

## 3.7 `TrajectoryPlanner` 这一阶段怎么改

虽然 `TrajectoryPlanner` 真正迁出 UI 是下一步重点，但 Phase 3 至少要让它开始使用新领域类型。

当前文件：
- [src/ui/TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h)
- [src/ui/TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp)

### 这一阶段最小修改目标

把内部输入输出逐步从：
- `nl::occ::Waypoint`
- `nl::occ::Trajectory`
- `gp_Trsf`
- `nl::utils::Q`

过渡到：
- `domain::Waypoint`
- `domain::Trajectory`
- `foundation::Pose`
- `foundation::JointState`

### 现实建议
Phase 3 不要一步改完所有函数签名，不然改动太大。  
建议分两层：

#### 第一层：头文件先切换到 domain 类型
例如在 [src/ui/TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h) 里把 `Trajectory` 和 `Waypoint` include 改成 domain 头。

#### 第二层：实现里先桥接
内部继续在局部用：
- `gp_Trsf trsf = ToGpTrsf(domain_pose);`
- `Q q = ToQ(joint_state);`

也就是说：
- 公共边界先改
- 算法内部后改

这是最稳的。

---

## 3.8 `MainWindow` 这一阶段怎么改

当前 [src/ui/MainWindow.h](E:/Code/GrindingApp/src/ui/MainWindow.h) 里有这些状态：

- `std::vector<nl::occ::Waypoint> waypoints_`
- `nl::occ::Trajectory trajectory_`

Phase 3 目标是把它们先改成：

```cpp
std::vector<domain::Waypoint> waypoints_;
domain::Trajectory trajectory_;
```

但如果你担心改太多，也可以过渡成：

- 头文件 include `domain/Waypoint.h`
- 头文件 include `domain/Trajectory.h`

然后用 `domain::...`

### `DisplayWaypoints()` 怎么办
当前 `DisplayWaypoints()` 用的是：

```cpp
const gp_XYZ& t = wp.pose.TranslationPart();
```

这个会因为 `wp.pose` 变成 `Pose` 而失效。  
解决方式：

```cpp
gp_Trsf trsf = foundation::ToGpTrsf(wp.pose);
const gp_XYZ& t = trsf.TranslationPart();
```

也就是说，UI 显示层允许桥接回 OCCT。

---

## 3.9 `WaypointGenerator` 这一阶段怎么改

当前 `WaypointGenerator` 很可能仍返回 `std::vector<nl::occ::Waypoint>`。  
Phase 3 有两种做法：

### 做法 A：先不动 generator 的实现，返回类型靠兼容别名
如果你已经把 [src/occ/Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h) 改成 `using Waypoint = domain::Waypoint;`，那么这一阶段它可能几乎不用改。

这是最推荐的过渡方案。

### 做法 B：直接改成 domain 类型
如果你想更彻底，可以把返回值直接改成：

```cpp
std::vector<domain::Waypoint>
```

但这样会带来更多 include 和命名空间联动修改。  
建议留到后续 `geometry` 模块正式整理时再做。

---

## 3.10 `Trajectory.h` 的旧字段名兼容问题

老代码里现在常用：

- `pt.joint_angles`
- `pt.status`
- `pt.move_type`
- `pt.tcp_pose`

新设计里如果你把 `joint_angles` 改成 `joint_state`，会引发较多改动。

### 建议：Phase 3 先保守一点
为了减小改动，Phase 3 可以临时保留旧字段名：

```cpp
struct TrajectoryPoint {
    foundation::Pose tcp_pose;
    foundation::JointState joint_angles;
    MoveType move_type = MoveType::kMoveL;
    TrajectoryPointStatus status = TrajectoryPointStatus::kOk;
    int waypoint_index = -1;
};
```

也就是说：

- 类型升级
- 名字先不升级

这样 `MainWindow.cpp`、`TrajectoryPlanner.cpp` 改动量会小很多。

后面等 Application 接管后，再把字段名统一调整成 `joint_state`。

这是一个很实用的过渡策略。

---

## 3.11 Phase 3 推荐的“低风险版本”

如果你要稳扎稳打，我建议 Phase 3 先这样落地：

### 新 domain 类型：
- `Waypoint` 用 `Pose`
- `TrajectoryPoint` 用 `Pose + JointState`
- 字段名尽量保持老名字

### 旧 occ 头：
- 做 `using` 别名兼容

### 旧 planner / mainwindow：
- 先继续工作
- 显示层通过 `ToGpTrsf()` 转回 OCCT
- 运动学层通过 `ToQ()` 转回旧 `Q`

这样你能先完成“归属纠正”，不强迫自己立刻做“算法内部重写”。

---

## 3.12 这阶段需要改哪些 CMake

### `src/domain/CMakeLists.txt`
新增

### `src/CMakeLists.txt`
加 `add_subdirectory(domain)`

### `src/occ/CMakeLists.txt`
需要让 `SSOcc` 链接 `SSDomain`

改成：

```cmake
target_link_libraries(SSOcc PUBLIC
    SSDomain
    SSCore
    SSKinematics
    ...
)
```

### `src/ui/CMakeLists.txt`
让 `SSUI` 也能直接 include domain：

```cmake
target_link_libraries(SSUI PUBLIC
    SSDomain
    SSFoundation
    SSCore
    SSOcc
    SSKinematics
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
)
```

### `src/kinematics/CMakeLists.txt`
如果后面很快会碰到 `JointState/Pose`，建议也链上：

```cmake
target_link_libraries(SSKinematics PUBLIC
    SSDomain
    SSFoundation
    SSCore
    ...
)
```

---

## 3.13 这阶段推荐的具体执行顺序

按这个顺序做最稳：

1. 创建 [src/domain/CMakeLists.txt](E:/Code/GrindingApp/src/domain/CMakeLists.txt)
2. 创建 [src/domain/Waypoint.h](E:/Code/GrindingApp/src/domain/Waypoint.h)
3. 创建 [src/domain/WaypointSet.h](E:/Code/GrindingApp/src/domain/WaypointSet.h)
4. 创建 [src/domain/Trajectory.h](E:/Code/GrindingApp/src/domain/Trajectory.h)
5. 修改 [src/CMakeLists.txt](E:/Code/GrindingApp/src/CMakeLists.txt) 增加 `domain`
6. 修改 [src/occ/CMakeLists.txt](E:/Code/GrindingApp/src/occ/CMakeLists.txt) 依赖 `SSDomain`
7. 修改 [src/ui/CMakeLists.txt](E:/Code/GrindingApp/src/ui/CMakeLists.txt) 依赖 `SSDomain` 和 `SSFoundation`
8. 把 [src/occ/Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h) 改成兼容别名
9. 把 [src/occ/Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h) 改成兼容别名
10. 先编译，确认兼容层可行
11. 再改 [src/ui/MainWindow.h](E:/Code/GrindingApp/src/ui/MainWindow.h) 的 `waypoints_` 和 `trajectory_` 引用
12. 再改 [src/ui/MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp) 中 `DisplayWaypoints()` 等显示桥接
13. 再改 [src/ui/TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h) 的 include 和类型引用
14. 再改 [src/ui/TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp) 的桥接点
15. 编译并运行主流程
16. 补测试

---

## 3.14 这阶段必须补的测试

建议新增：

- [tests/TestDomainModels.cpp](E:/Code/GrindingApp/tests/TestDomainModels.cpp)

至少测这些：

### `Trajectory::HasErrors`
- 所有点都 `kOk` 时返回 `false`
- 任一点 `kIkFailed` 时返回 `true`

### `Trajectory::ErrorCount`
- 不同状态组合能正确计数

### `Pose` 搭配 `Waypoint`
- `Waypoint` 默认值正确
- `speed_ratio` 默认 1.0

### `JointState` 搭配 `TrajectoryPoint`
- `joint_angles.values_deg` 可正常保存和读取

如果你不想新建测试文件，最低限度也应该在现有测试中补一个 domain model smoke test。

---

## 3.15 Phase 3 完成后的硬性验收标准

做到下面这些，就算 Phase 3 完成：

- `Waypoint` 的主定义已在 `domain`
- `Trajectory` 的主定义已在 `domain`
- `occ` 里的对应头文件只剩兼容转发或别名
- `MainWindow` 和 `TrajectoryPlanner` 已能使用新 domain 类型
- 现有主流程仍能运行
- 编译不回归
- 至少有基础模型测试覆盖 `Trajectory::HasErrors/ErrorCount`

---

## 3.16 Phase 3 完成后，下一步就该做什么

完成这一步后，下一阶段最自然的是：

1. 把 [src/ui/TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp) 真正迁到 `planning`
2. 建立 `IKinematicsService`
3. 建立 `ITrajectoryPlanner`
4. 开始让 `MainWindow` 不再直接依赖底层算法

也就是说，Phase 3 是在为“Planning 从 UI 脱身”铺路。

如果你愿意，我下一条可以继续给你：
**Phase 3 的代码修改清单，直接列到“哪些 include 要改、哪些字段要改、哪些函数体要桥接”，尽量贴近你当前仓库。**