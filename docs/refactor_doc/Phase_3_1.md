下面这份就是贴着你当前仓库的 `Phase 3` 逐文件修改清单。我按“先能编译、再逐步切类型”的思路写，尽量避免你一次改爆。

**总策略**

`Phase 3` 不做大规模逻辑搬家，只做三件事：

- 在 `domain` 建立 `Waypoint / WaypointSet / Trajectory`
- 让 `occ` 里的同名头退化成兼容层
- 让 `MainWindow` 和 `TrajectoryPlanner` 开始使用新 domain 类型，并在函数体里桥接回 `gp_Trsf` / `Q`

---

**一、要新增的文件**

先创建这些文件：

- [src/domain/CMakeLists.txt](E:/Code/GrindingApp/src/domain/CMakeLists.txt)
- [src/domain/Waypoint.h](E:/Code/GrindingApp/src/domain/Waypoint.h)
- [src/domain/WaypointSet.h](E:/Code/GrindingApp/src/domain/WaypointSet.h)
- [src/domain/Trajectory.h](E:/Code/GrindingApp/src/domain/Trajectory.h)

建议内容我按你当前仓库的兼容需求做最小版。

[src/domain/Waypoint.h](E:/Code/GrindingApp/src/domain/Waypoint.h)

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

[src/domain/WaypointSet.h](E:/Code/GrindingApp/src/domain/WaypointSet.h)

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

[src/domain/Trajectory.h](E:/Code/GrindingApp/src/domain/Trajectory.h)

这里建议先保留旧字段名 `joint_angles`，这样 `MainWindow.cpp` 和 `TrajectoryPlanner.cpp` 改动会小很多。

```cpp
#pragma once

#include <vector>

#include "Ids.h"
#include "Pose.h"
#include "JointState.h"

namespace domain {

struct TrajectoryPoint {
    foundation::Pose tcp_pose;
    foundation::JointState joint_angles;

    enum class MoveType { kMoveJ, kMoveL };
    MoveType move_type = MoveType::kMoveL;

    enum class Status { kOk, kIkFailed, kJointJump, kCollision };
    Status status = Status::kOk;

    int waypoint_index = -1;
};

struct Trajectory {
    foundation::TrajectoryId id;
    std::vector<TrajectoryPoint> points;

    bool HasErrors() const {
        for (const auto& p : points) {
            if (p.status != TrajectoryPoint::Status::kOk) {
                return true;
            }
        }
        return false;
    }

    int ErrorCount() const {
        int count = 0;
        for (const auto& p : points) {
            if (p.status != TrajectoryPoint::Status::kOk) {
                ++count;
            }
        }
        return count;
    }
};

} // namespace domain
```

[src/domain/CMakeLists.txt](E:/Code/GrindingApp/src/domain/CMakeLists.txt)

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

---

**二、CMake 需要改哪些地方**

**1. 修改 [src/CMakeLists.txt](E:/Code/GrindingApp/src/CMakeLists.txt)**

把 `domain` 加进去，建议顺序放在 `foundation` 后面：

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

**2. 修改 [src/occ/CMakeLists.txt](E:/Code/GrindingApp/src/occ/CMakeLists.txt)**

在 `target_link_libraries(SSOcc PUBLIC ...)` 里加：

```cmake
SSDomain
SSFoundation
```

推荐放在最前面：

```cmake
target_link_libraries(SSOcc PUBLIC
    SSDomain
    SSFoundation
    SSCore
    SSKinematics
    ...
)
```

**3. 修改 [src/ui/CMakeLists.txt](E:/Code/GrindingApp/src/ui/CMakeLists.txt)**

同样加：

```cmake
SSDomain
SSFoundation
```

例如：

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

**4. 修改 [src/kinematics/CMakeLists.txt](E:/Code/GrindingApp/src/kinematics/CMakeLists.txt)**

如果你马上就会在规划桥接里用 `Pose/JointState`，建议现在就加：

```cmake
SSDomain
SSFoundation
```

---

**三、把 `occ` 头改成兼容层**

这是 `Phase 3` 最关键的“低风险手法”。

**1. 修改 [src/occ/Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h)**

原文件删掉旧定义，改成：

```cpp
#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_H_

#include "domain/Waypoint.h"

namespace nl {
namespace occ {

using Waypoint = domain::Waypoint;

} // namespace occ
} // namespace nl

#endif  // GRINDINGAPP_SRC_OCC_WAYPOINT_H_
```

**2. 修改 [src/occ/Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h)**

改成：

```cpp
#ifndef GRINDINGAPP_SRC_OCC_TRAJECTORY_H_
#define GRINDINGAPP_SRC_OCC_TRAJECTORY_H_

#include "domain/Trajectory.h"

namespace nl {
namespace occ {

using Trajectory = domain::Trajectory;
using TrajectoryPoint = domain::TrajectoryPoint;

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_TRAJECTORY_H_
```

这样做完以后：

- `occ::Waypoint` 其实已经是 `domain::Waypoint`
- `occ::Trajectory` 其实已经是 `domain::Trajectory`

所以很多旧代码先不用全改，也能继续编译。

---

**四、Foundation 里需要补哪些桥接**

`Phase 3` 要正式开始在 UI/Planner 里桥接 `Pose <-> gp_Trsf`、`JointState <-> Q`。

你现有 [src/foundation/Conversions.h](E:/Code/GrindingApp/src/foundation/Conversions.h) 和 [src/foundation/Conversions.cpp](E:/Code/GrindingApp/src/foundation/Conversions.cpp) 需要已经有这 4 个函数：

- `Pose ToPose(const gp_Trsf& trsf);`
- `gp_Trsf ToGpTrsf(const Pose& pose);`
- `JointState ToJointState(const nl::utils::Q& q);`
- `nl::utils::Q ToQ(const JointState& joints);`

如果还没有，这一步必须先补齐。

---

**五、[src/ui/TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h) 怎么改**

这个文件是 `Phase 3` 的重点。

当前 include：

- `"Trajectory.h"`
- `"Waypoint.h"`

这两个旧头虽然会被兼容层兜住，但建议你开始显式依赖 domain。

**修改建议**

**1. include 改成：**

```cpp
#include <vector>
#include <gp_Trsf.hxx>

#include "domain/Trajectory.h"
#include "domain/Waypoint.h"
#include "RbXmlParser.h"
#include "SSUIExport.h"
```

这里要补 `gp_Trsf.hxx`，因为头文件里用了 `gp_Trsf`。

**2. `Plan` 的签名改成：**

```cpp
domain::Trajectory Plan(
    const std::vector<domain::Waypoint>& waypoints,
    const nl::core::RbRobot& robot,
    const nl::utils::Q& current_angles,
    const Config& config);
```

**3. `ResolveSinglePoint` 改成：**

```cpp
bool ResolveSinglePoint(
    domain::Trajectory& traj, int index,
    const nl::core::RbRobot& robot,
    int solution_index);
```

**4. 私有函数返回值改成：**

```cpp
std::vector<domain::TrajectoryPoint> InterpolateMoveJ(...)
std::vector<domain::TrajectoryPoint> InterpolateMoveL(...)
```

**5. 这个阶段先不改的东西**

- `const nl::utils::Q& from/to`
- `const gp_Trsf& target_pose`
- `const gp_Trsf& from_pose/to_pose`

这些先保留，原因是 `Phase 3` 只改“边界类型”，算法内部先不大动。

---

**六、[src/ui/TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp) 怎么桥接**

这个文件里要改的主要是 `Pose` 和 `JointState` 的桥接。

**1. 顶部 include 补：**

```cpp
#include "Conversions.h"
```

如果路径解析有问题，就用你实际可 include 的方式，比如：
- `"foundation/Conversions.h"`
- 或继续走项目现有 include 方式

**2. 顶部 using 语句建议改成：**

```cpp
using domain::Trajectory;
using domain::TrajectoryPoint;
using domain::Waypoint;
```

不要再 `using nl::occ::...`

**3. `InterpolateMoveJ()` 里改这一行**

当前：

```cpp
pt.joint_angles = nl::utils::Q(n);
```

改成：

```cpp
nl::utils::Q q(n);
for (int j = 0; j < n; ++j) {
    q[j] = from[j] * (1.0 - t) + to[j] * t;
}
pt.joint_angles = foundation::ToJointState(q);
```

同时删掉旧的直接给 `pt.joint_angles[j] = ...` 的写法，因为新类型不是 `Q`。

**4. `InterpolateMoveJ()` 里改 `tcp_pose` 赋值**

当前：

```cpp
pt.tcp_pose = target_pose;
```

改成：

```cpp
pt.tcp_pose = foundation::ToPose(target_pose);
```

**5. `InterpolateMoveL()` 里改 `tcp_pose`**

当前：

```cpp
pt.tcp_pose = InterpolatePose(from_pose, to_pose, t);
```

改成两步：

```cpp
gp_Trsf tcp_pose = InterpolatePose(from_pose, to_pose, t);
pt.tcp_pose = foundation::ToPose(tcp_pose);
```

**6. `InterpolateMoveL()` 里 IK 调用要桥接**

当前：

```cpp
nl::utils::Q ik_result;
bool ok = nl::kinematics::ComputeIk(
    robot, pt.tcp_pose, prev_angles, ik_result);
```

这里 `pt.tcp_pose` 已经是 `Pose`，不能直接传。改成：

```cpp
nl::utils::Q ik_result;
gp_Trsf tcp_pose_trsf = foundation::ToGpTrsf(pt.tcp_pose);
bool ok = nl::kinematics::ComputeIk(
    robot, tcp_pose_trsf, prev_angles, ik_result);
```

**7. `InterpolateMoveL()` 里 joint_angles 赋值改桥接**

当前失败分支：

```cpp
pt.joint_angles = prev_angles;
```

改成：

```cpp
pt.joint_angles = foundation::ToJointState(prev_angles);
```

当前成功分支：

```cpp
pt.joint_angles = ik_result;
```

改成：

```cpp
pt.joint_angles = foundation::ToJointState(ik_result);
```

`prev_angles` 本地变量仍然保持 `Q` 不动，这样算法改动最小。

**8. `Plan()` 中处理 `prev_angles` 的地方要桥接**

当前：

```cpp
prev_angles = last.joint_angles;
```

改成：

```cpp
prev_angles = foundation::ToQ(last.joint_angles);
```

**9. `ResolveSinglePoint()` 里桥接**

当前：

```cpp
seed = traj.points[index - 1].joint_angles;
```

改成：

```cpp
seed = foundation::ToQ(traj.points[index - 1].joint_angles);
```

当前：

```cpp
seed = pt.joint_angles;
```

改成：

```cpp
seed = foundation::ToQ(pt.joint_angles);
```

当前：

```cpp
robot, pt.tcp_pose, seed, solutions
```

改成：

```cpp
robot, foundation::ToGpTrsf(pt.tcp_pose), seed, solutions
```

当前：

```cpp
pt.joint_angles = solutions[solution_index];
```

改成：

```cpp
pt.joint_angles = foundation::ToJointState(solutions[solution_index]);
```

**10. `HasJointJump()` 这个阶段先不动**

因为它还只接受 `Q`，而你在 `ResolveSinglePoint()` 里可以桥接回 `Q` 后再调用。

---

**七、[src/ui/MainWindow.h](E:/Code/GrindingApp/src/ui/MainWindow.h) 怎么改**

这个头文件要先开始显式依赖 domain。

**1. include 改法**

当前：

```cpp
#include "Waypoint.h"
#include "IWaypointAlgo.h"
#include "WaypointGenerator.h"
#include "Trajectory.h"
#include "TrajectoryPlanner.h"
```

建议改成：

```cpp
#include "domain/Waypoint.h"
#include "domain/Trajectory.h"
#include "IWaypointAlgo.h"
#include "WaypointGenerator.h"
#include "TrajectoryPlanner.h"
```

`IWaypointAlgo.h` 和 `WaypointGenerator.h` 暂时还保留，因为它们还在 `occ`。

**2. 字段类型改法**

把：

```cpp
std::vector<nl::occ::Waypoint>  waypoints_;
```

改成：

```cpp
std::vector<domain::Waypoint>  waypoints_;
```

把：

```cpp
nl::occ::Trajectory trajectory_;
```

改成：

```cpp
domain::Trajectory trajectory_;
```

**3. `waypoint_gen_` 和 `waypoint_config_` 暂时不动**

因为这俩还在 `occ`，留给后续 phase。

---

**八、[src/ui/MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp) 哪些函数体要桥接**

这个文件改动会比较多，但都很机械。

**1. 顶部 include 补：**

```cpp
#include "Conversions.h"
```

**2. `OnGenerateWaypoints()` 的 world pose 叠加要桥接**

当前：

```cpp
for (auto& wp : waypoints_) {
    gp_Trsf world_pose = workpiece_trsf_;
    world_pose.Multiply(wp.pose);
    wp.pose = world_pose;
}
```

改成：

```cpp
for (auto& wp : waypoints_) {
    gp_Trsf world_pose = workpiece_trsf_;
    world_pose.Multiply(foundation::ToGpTrsf(wp.pose));
    wp.pose = foundation::ToPose(world_pose);
}
```

**3. `DisplayWaypoints()` 要桥接**

当前：

```cpp
const gp_XYZ& t = wp.pose.TranslationPart();
```

改成：

```cpp
gp_Trsf trsf = foundation::ToGpTrsf(wp.pose);
const gp_XYZ& t = trsf.TranslationPart();
```

**4. `OnPlanTrajectory()` 里 `base_wps` 类型改成 domain**

当前：

```cpp
std::vector<nl::occ::Waypoint> base_wps = waypoints_;
```

改成：

```cpp
std::vector<domain::Waypoint> base_wps = waypoints_;
```

**5. `OnPlanTrajectory()` 里每个 waypoint 的 pose 变换要桥接**

当前：

```cpp
gp_Trsf t = base_inv;
t.Multiply(wp.pose);
t.Multiply(tcp_inv);
wp.pose = t;
```

改成：

```cpp
gp_Trsf t = base_inv;
t.Multiply(foundation::ToGpTrsf(wp.pose));
t.Multiply(tcp_inv);
wp.pose = foundation::ToPose(t);
```

**6. `OnTrajectoryPointSelected()` 里 joint_angles 和 tcp_pose 要桥接**

当前判断：

```cpp
if (pt.status != nl::occ::TrajectoryPoint::Status::kIkFailed) {
    controller_->SetJointAngles(pt.joint_angles);
}
```

改成：

```cpp
if (pt.status != domain::TrajectoryPoint::Status::kIkFailed) {
    controller_->SetJointAngles(foundation::ToQ(pt.joint_angles));
}
```

当前 IK 求解：

```cpp
nl::kinematics::ComputeIkAllSolutions(
    controller_->GetRobot(), pt.tcp_pose,
    pt.joint_angles, solutions);
```

改成：

```cpp
nl::kinematics::ComputeIkAllSolutions(
    controller_->GetRobot(),
    foundation::ToGpTrsf(pt.tcp_pose),
    foundation::ToQ(pt.joint_angles),
    solutions);
```

**7. `OnIkSolutionChanged()` 里 controller 设置角度要桥接**

当前：

```cpp
controller_->SetJointAngles(trajectory_.points[point_index].joint_angles);
```

改成：

```cpp
controller_->SetJointAngles(
    foundation::ToQ(trajectory_.points[point_index].joint_angles));
```

**8. `OnPlaybackFrame()` 要桥接**

当前：

```cpp
if (pt.status == nl::occ::TrajectoryPoint::Status::kIkFailed)
    return;

controller_->SetJointAngles(pt.joint_angles);
```

改成：

```cpp
if (pt.status == domain::TrajectoryPoint::Status::kIkFailed)
    return;

controller_->SetJointAngles(foundation::ToQ(pt.joint_angles));
```

**9. `TransformWaypointsAndTrajectory()` 要桥接 waypoint pose**

当前：

```cpp
gp_Trsf t = delta;
t.Multiply(wp.pose);
wp.pose = t;
```

改成：

```cpp
gp_Trsf t = delta;
t.Multiply(foundation::ToGpTrsf(wp.pose));
wp.pose = foundation::ToPose(t);
```

**10. 这阶段先不动的成员**

这些暂时保留原状：

- `TopoDS_Shape workpiece_shape_`
- `TopoDS_Face selected_face_`
- `gp_Trsf workpiece_trsf_`
- `current_ik_solutions_`

因为它们属于后面 geometry/application 继续拆的范围，不是 `Phase 3` 的重点。

---

**九、[src/occ/IWaypointAlgo.h](E:/Code/GrindingApp/src/occ/IWaypointAlgo.h) 这阶段怎么处理**

这个文件现在包含 `"Waypoint.h"`。  
由于你已经把 [src/occ/Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h) 改成 domain 别名，这里理论上可以先不改。

但我建议你至少做一件事：

**在注释里标记它是过渡接口**

比如加一句：

```cpp
// Transitional interface: Waypoint is currently an alias to domain::Waypoint.
```

这样以后你自己不会忘。

函数签名先不改：

```cpp
virtual std::vector<Waypoint> Generate(
    const TopoDS_Face& face,
    const WaypointConfig& config) = 0;
```

因为 `Waypoint` 已经变成 domain 别名了。

---

**十、[src/occ/WaypointGenerator.h](E:/Code/GrindingApp/src/occ/WaypointGenerator.h) 这阶段怎么处理**

同理，它的返回值先不改：

```cpp
std::vector<Waypoint> Generate(const WaypointConfig& config);
```

因为 `Waypoint` 已经是兼容别名。

这能让你 Phase 3 只改 domain 归属，不在 geometry 这一层起大火。

---

**十一、推荐你直接搜索替换的关键模式**

你在改的时候可以先全局搜这些模式，改动会很快：

**1. 搜 `std::vector<nl::occ::Waypoint>`**
主要改成：
- `std::vector<domain::Waypoint>`

**2. 搜 `nl::occ::Trajectory`**
主要改成：
- `domain::Trajectory`

**3. 搜 `pt.joint_angles`**
看场景：
- 如果传给 `controller_->SetJointAngles()`，就包 `ToQ(...)`
- 如果传给 `ComputeIkAllSolutions()`，也包 `ToQ(...)`

**4. 搜 `pt.tcp_pose`**
看场景：
- 如果传给 `ComputeIk/ComputeIkAllSolutions()`，就包 `ToGpTrsf(...)`

**5. 搜 `wp.pose`**
看场景：
- 如果要做 OCCT 变换或取平移，先 `ToGpTrsf(wp.pose)`
- 如果要回写，包 `ToPose(...)`

---

**十二、这阶段最容易编译不过的点**

你大概率会遇到这几类错误：

**1. `gp_Trsf` 头没包含**
尤其是改了 include 之后，[src/ui/TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h) 需要显式 include `<gp_Trsf.hxx>`。

**2. `domain/xxx.h` include 路径找不到**
如果报这个错，说明：
- `SSDomain` 的 include dir 没配对
- 或者你项目里仍按“顶层 `src` 统一 include”用法，需要调整 include 写法

如果你当前项目更习惯直接 `#include "Waypoint.h"` 风格，那过渡期也可以先写：
- `#include "domain/Trajectory.h"` 失败时，改成一个项目可见路径
- 关键是先统一目标模块，不必执着于 include 样式

**3. `JointState` 没有 `operator[]`**
如果你没按我前面建议在 [src/foundation/JointState.h](E:/Code/GrindingApp/src/foundation/JointState.h) 里加 `operator[]`，有些代码会不方便。建议加上。

**4. `TrajectoryPoint::Status` 命名空间不一致**
因为你从 `nl::occ::TrajectoryPoint::Status` 切到了 `domain::TrajectoryPoint::Status`，要逐处改。

---

**十三、这阶段你先不要碰的文件**

为了控制风险，下面这些这轮最好不动或少动：

- [src/ui/RobotController.h](E:/Code/GrindingApp/src/ui/RobotController.h)
- [src/ui/RobotController.cpp](E:/Code/GrindingApp/src/ui/RobotController.cpp)
- [src/kinematics/RobotKinematics.cpp](E:/Code/GrindingApp/src/kinematics/RobotKinematics.cpp)
- [src/core/RbXmlParser.h](E:/Code/GrindingApp/src/core/RbXmlParser.h)
- [src/core/RbXmlParser.cpp](E:/Code/GrindingApp/src/core/RbXmlParser.cpp)

这几个属于后面 `Planning/Application/Import` phase 的重点，不值得在 `Phase 3` 一起搅进去。

---

**十四、Phase 3 最推荐的实际落地顺序**

按这个顺序改，最稳：

1. 新建 `domain` 目录和 3 个头文件
2. 改 [src/CMakeLists.txt](E:/Code/GrindingApp/src/CMakeLists.txt)
3. 改 [src/occ/CMakeLists.txt](E:/Code/GrindingApp/src/occ/CMakeLists.txt)
4. 改 [src/ui/CMakeLists.txt](E:/Code/GrindingApp/src/ui/CMakeLists.txt)
5. 把 [src/occ/Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h) 改成兼容层
6. 把 [src/occ/Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h) 改成兼容层
7. 编译一次，确认兼容层没把项目炸掉
8. 改 [src/ui/TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h)
9. 改 [src/ui/TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp)
10. 改 [src/ui/MainWindow.h](E:/Code/GrindingApp/src/ui/MainWindow.h)
11. 改 [src/ui/MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp)
12. 编译
13. 跑主流程
14. 补测试

---

**十五、完成后的验收点**

你改完这一轮后，至少确认这些：

- 能编译
- 导入工件后还能选面
- 还能生成 waypoint
- 还能规划 trajectory
- 轨迹点选中后，机器人还能跳到对应姿态
- playback 还能跑
- `Waypoint` 和 `Trajectory` 的主定义已经在 `domain`
- `occ` 的同名头只剩兼容作用

如果你要，我下一条可以继续直接给你：
**[src/ui/TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp) 和 [src/ui/MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp) 的“逐函数修改版伪 diff”，会更接近你可以直接动手改的形式。**