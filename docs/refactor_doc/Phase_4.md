# Phase 4：把 TrajectoryPlanner 从 UI 抽到 Planning

## 背景

Phase 3 已完成（commit 617735d）：Waypoint / Trajectory 迁移到 domain 层。
Phase 4 目标：把 `nl::ui::TrajectoryPlanner` 搬出 GrindingUI，放进 SSPlanning 模块，
同时建立 `ITrajectoryPlanner` 抽象接口。

完成后 UI 模块不再包含任何规划算法，`TrajectoryPlanner` 不再属于 GrindingUI target。

---

## 修改边界

### 新增文件
| 文件 | 说明 |
|---|---|
| `src/planning/GrindingPlanningExport.h` | DLL 导出宏 |
| `src/planning/CartesianTrajectoryPlanner.h` | 具体实现类声明 |
| `src/planning/CartesianTrajectoryPlanner.cpp` | 具体实现类定义 |

### 修改文件
| 文件 | 修改内容 |
|---|---|
| `src/planning/PlanningTypes.h` | 填充 `PlanningRequest` 结构体 |
| `src/planning/ITrajectoryPlanner.h` | 填充接口定义 |
| `src/planning/CMakeLists.txt` | INTERFACE → SHARED，加源文件和依赖 |
| `src/ui/CMakeLists.txt` | 删除 TrajectoryPlanner.h/.cpp，加 SSPlanning 依赖 |
| `src/ui/MainWindow.h` | 替换 `TrajectoryPlanner` → `planning::CartesianTrajectoryPlanner` |
| `src/ui/MainWindow.cpp` | 适配新返回值 `Result<Trajectory>`，不再临时构造 |
| `docs/architecture.md` | 更新模块表 |

### 删除文件
- `src/ui/TrajectoryPlanner.h`
- `src/ui/TrajectoryPlanner.cpp`

---

## 设计决策

| 决策点 | 选择 | 原因 |
|---|---|---|
| Robot 参数类型 | 保持 `nl::core::RbRobot` | domain::Robot 尚未建立，过渡期保持稳定 |
| `ResolveSinglePoint` 位置 | 只在具体类，不进接口 | 是 UI 交互行为，非规划契约；Phase 5 建好 IKinematicsService 后再重构 |
| 返回类型 | `foundation::Result<domain::Trajectory>` | 按设计文档；空路径点时返回 Fail |
| 接口参数 | `domain::WaypointSet`（Phase 3 已建好） | 不用 `vector<Waypoint>`，对齐设计文档 |
| `PlanningSceneSnapshot` | **不引入**，留 Phase 7 Step 25 | AppService 层尚未建好 |
| 命名空间 | `planning::` 单层 | 与 `domain::` / `foundation::` 一致 |
| DLL 导出宏 | `__declspec` 直接实现，不用 Qt 宏 | CartesianTrajectoryPlanner 无 Qt 依赖 |
| 私有 helper 位置 | 移到 `.cpp` 匿名 namespace | private 方法用 `gp_Trsf`（第三方类型），不能出现在头文件 |

---

## 完整代码

### 1. `src/planning/GrindingPlanningExport.h`（新建）

```cpp
#pragma once

#if defined(_WIN32)
    #ifdef GRINDING_PLANNING_BUILDING_DLL
        #define GRINDING_PLANNING_EXPORT __declspec(dllexport)
    #else
        #define GRINDING_PLANNING_EXPORT __declspec(dllimport)
    #endif
#else
    #define GRINDING_PLANNING_EXPORT
#endif

```

---

### 2. `src/planning/PlanningTypes.h`（填充，原来空文件）

```cpp
#pragma once

namespace planning {

struct PlanningRequest {
    double approach_dist = 50.0;
    int movej_steps = 50;
    int movel_steps_per_seg = 10;
    double joint_jump_threshold = 30.0;
};

} // namespace planning

```

---

### 3. `src/planning/ITrajectoryPlanner.h`（填充，原来空文件）

> **[Strategy 模式]** 抽象策略接口：定义"规划"这件事的契约，不绑定任何具体算法。

```cpp
#pragma once

#include "domain/Trajectory.h"
#include "domain/WaypointSet.h"
#include "foundation/Result.h"
#include "PlanningTypes.h"
#include "RbXmlParser.h"
#include "Q.h"

namespace planning {

// [Strategy] 抽象策略：规定接口契约，不含任何算法实现
class ITrajectoryPlanner {
public:
    virtual ~ITrajectoryPlanner() = default;

    // [Result 惯用法] 返回 Result<T> 而非抛异常或返回空对象，
    //   调用方被迫显式处理失败，错误信息有类型、有描述
    virtual foundation::Result<domain::Trajectory> Plan(
        const nl::core::RbRobot& robot,
        const nl::utils::Q& current_angles,
        const domain::WaypointSet& waypoints,
        const PlanningRequest& request) = 0;
};

} // namespace planning

```

---

### 4. `src/planning/CartesianTrajectoryPlanner.h`（新建）

> **[Strategy 模式]** 具体策略：实现笛卡尔空间插值规划算法，可被其他实现替换。

```cpp
#pragma once

#include "ITrajectoryPlanner.h"
#include "GrindingPlanningExport.h"

namespace planning {

// [Strategy] 具体策略：笛卡尔插值规划（approach + MoveJ + MoveL + IK + jump 检查）
class GRINDING_PLANNING_EXPORT CartesianTrajectoryPlanner : public ITrajectoryPlanner {
public:
    // [Facade] 对外一个入口，内部封装 5 步规划流程
    foundation::Result<domain::Trajectory> Plan(
        const nl::core::RbRobot& robot,
        const nl::utils::Q& current_angles,
        const domain::WaypointSet& waypoints,
        const PlanningRequest& request) override;

    // 不在接口内：UI 交互行为，用于面板重选 IK 解
    bool ResolveSinglePoint(
        domain::Trajectory& traj,
        int index,
        const nl::core::RbRobot& robot,
        int solution_index);
};

} // namespace planning

```

---

### 5. `src/planning/CartesianTrajectoryPlanner.cpp`（新建）

私有 helper 全部移入匿名 namespace，避免 OCCT 类型污染头文件。

> **[Pimpl 精神]** 没用完整 Pimpl，但把所有依赖 `gp_Trsf` 的 helper 函数沉入 `.cpp` 匿名
> namespace，消费方编译时完全看不到 OCCT。效果与 Pimpl 等价：头文件零第三方类型。

```cpp
#include "CartesianTrajectoryPlanner.h"

#include <cmath>

// OCCT 类型只出现在 .cpp，不污染头文件 —— [Pimpl 精神]
#include <gp_Vec.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>

#include "RobotKinematics.h"
#include "Q.h"
#include "Conversions.h"

using domain::Trajectory;
using domain::TrajectoryPoint;
using domain::Waypoint;

// [Pimpl 精神] 所有依赖 gp_Trsf 的 helper 放匿名 namespace，对外完全不可见
namespace {

gp_Trsf ComputeApproachPose(const gp_Trsf& first_wp_pose, double approach_dist)
{
    gp_XYZ z_col(first_wp_pose.Value(1, 3),
                  first_wp_pose.Value(2, 3),
                  first_wp_pose.Value(3, 3));
    gp_XYZ origin = first_wp_pose.TranslationPart();
    gp_XYZ approach_origin = origin - z_col * approach_dist;

    gp_Trsf result = first_wp_pose;
    result.SetTranslationPart(gp_Vec(approach_origin));
    return result;
}

bool HasJointJump(const nl::utils::Q& a, const nl::utils::Q& b, double threshold)
{
    int n = std::min(a.size(), b.size());
    for (int i = 0; i < n; ++i) {
        if (std::abs(a[i] - b[i]) > threshold)
            return true;
    }
    return false;
}

std::vector<TrajectoryPoint> InterpolateMoveJ(
    const nl::utils::Q& from,
    const nl::utils::Q& to,
    const gp_Trsf& target_pose,
    int steps)
{
    std::vector<TrajectoryPoint> result;
    if (steps < 2) steps = 2;

    int n = std::min(from.size(), to.size());
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;

        TrajectoryPoint pt;
        pt.move_type = domain::MoveType::kMoveJ;
        nl::utils::Q q(n);
        for (int j = 0; j < n; ++j) {
            q[j] = from[j] * (1.0 - t) + to[j] * t;
        }
        pt.joint_state = foundation::ToJointState(q);
        pt.tcp_pose = foundation::ToPose(target_pose);
        pt.status = domain::TrajectoryPointStatus::kOk;
        result.push_back(pt);
    }
    return result;
}

gp_Trsf InterpolatePose(const gp_Trsf& from, const gp_Trsf& to, double t)
{
    gp_XYZ p0 = from.TranslationPart();
    gp_XYZ p1 = to.TranslationPart();
    gp_XYZ pos = p0 * (1.0 - t) + p1 * t;

    gp_Quaternion q0 = from.GetRotation();
    gp_Quaternion q1 = to.GetRotation();

    gp_Quaternion q0_inv = q0.Inverted();
    gp_Quaternion q_diff = q0_inv * q1;

    double dot = q0.X() * q1.X() + q0.Y() * q1.Y() +
                 q0.Z() * q1.Z() + q0.W() * q1.W();
    if (dot < 0.0) {
        q1.Set(-q1.X(), -q1.Y(), -q1.Z(), -q1.W());
        q_diff = q0_inv * q1;
    }

    double angle = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.W())));
    gp_Quaternion qi;
    if (angle < 1e-10) {
        qi = q0;
    } else {
        double sin_half = std::sin(angle / 2.0);
        double ax = q_diff.X() / sin_half;
        double ay = q_diff.Y() / sin_half;
        double az = q_diff.Z() / sin_half;
        double scaled_angle = angle * t;
        double sin_s = std::sin(scaled_angle / 2.0);
        double cos_s = std::cos(scaled_angle / 2.0);
        gp_Quaternion q_part(ax * sin_s, ay * sin_s, az * sin_s, cos_s);
        qi = q0 * q_part;
    }

    gp_Trsf result;
    result.SetRotation(qi);
    result.SetTranslationPart(gp_Vec(pos));
    return result;
}

std::vector<TrajectoryPoint> InterpolateMoveL(
    const gp_Trsf& from_pose,
    const gp_Trsf& to_pose,
    const nl::core::RbRobot& robot,
    const nl::utils::Q& seed_angles,
    int steps,
    int waypoint_index)
{
    std::vector<TrajectoryPoint> result;
    if (steps < 1) steps = 1;

    nl::utils::Q prev_angles = seed_angles;

    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;

        TrajectoryPoint pt;
        pt.move_type = domain::MoveType::kMoveL;
        gp_Trsf tcp_pose = InterpolatePose(from_pose, to_pose, t);
        pt.tcp_pose = foundation::ToPose(tcp_pose);
        if (i == steps) {
            pt.waypoint_index = waypoint_index;
        }

        nl::utils::Q ik_result;
        gp_Trsf tcp_pose_trsf = foundation::ToGpTrsf(pt.tcp_pose);
        bool ok = nl::kinematics::ComputeIk(robot, tcp_pose_trsf, prev_angles, ik_result);

        if (!ok) {
            pt.status = domain::TrajectoryPointStatus::kIkFailed;
            pt.joint_state = foundation::ToJointState(prev_angles);
        } else {
            pt.joint_state = foundation::ToJointState(ik_result);
            pt.status = domain::TrajectoryPointStatus::kOk;
            prev_angles = ik_result;
        }

        result.push_back(pt);
    }
    return result;
}

} // anonymous namespace

namespace planning {

// [Facade] 调用方只需调一个 Plan()，内部顺序执行：
//   1. 计算 approach pose
//   2. MoveJ 插值到 approach
//   3. 逐段 MoveL 插值 + IK 求解
//   4. joint jump 后处理检查
// [Result 惯用法] 失败时返回 Fail，成功时 Ok 包裹结果
foundation::Result<domain::Trajectory> CartesianTrajectoryPlanner::Plan(
    const nl::core::RbRobot& robot,
    const nl::utils::Q& current_angles,
    const domain::WaypointSet& waypoints,
    const PlanningRequest& request)
{
    if (waypoints.points.empty()) {
        return foundation::Result<domain::Trajectory>::Fail(
            foundation::Error{"CartesianTrajectoryPlanner: no waypoints"});
    }

    Trajectory traj;

    gp_Trsf approach_pose = ComputeApproachPose(
        foundation::ToGpTrsf(waypoints.points[0].pose), request.approach_dist);

    nl::utils::Q approach_q;
    bool approach_ok = nl::kinematics::ComputeIk(
        robot, approach_pose, current_angles, approach_q);

    if (approach_ok) {
        auto movej_pts = InterpolateMoveJ(
            current_angles, approach_q, approach_pose, request.movej_steps);
        for (auto& pt : movej_pts)
            traj.points.push_back(std::move(pt));
    } else {
        TrajectoryPoint pt;
        pt.tcp_pose = foundation::ToPose(approach_pose);
        pt.joint_state = foundation::ToJointState(current_angles);
        pt.move_type = domain::MoveType::kMoveJ;
        pt.status = domain::TrajectoryPointStatus::kIkFailed;
        traj.points.push_back(pt);
    }

    nl::utils::Q prev_angles = approach_ok ? approach_q : current_angles;
    gp_Trsf prev_pose = approach_pose;

    for (size_t i = 0; i < waypoints.points.size(); ++i) {
        auto movel_pts = InterpolateMoveL(
            prev_pose,
            foundation::ToGpTrsf(waypoints.points[i].pose),
            robot,
            prev_angles,
            request.movel_steps_per_seg,
            static_cast<int>(i));

        for (auto& pt : movel_pts)
            traj.points.push_back(std::move(pt));

        if (!traj.points.empty()) {
            const auto& last = traj.points.back();
            if (last.status == domain::TrajectoryPointStatus::kOk)
                prev_angles = foundation::ToQ(last.joint_state);
            prev_pose = foundation::ToGpTrsf(waypoints.points[i].pose);
        }
    }

    // Post-process: detect joint jumps
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& pt = traj.points[i];
        if (pt.status != domain::TrajectoryPointStatus::kOk) continue;
        const auto& prev = traj.points[i - 1];
        if (prev.status != domain::TrajectoryPointStatus::kOk) continue;
        if (HasJointJump(
                foundation::ToQ(prev.joint_state),
                foundation::ToQ(pt.joint_state),
                request.joint_jump_threshold)) {
            pt.status = domain::TrajectoryPointStatus::kJointJump;
        }
    }

    return foundation::Result<domain::Trajectory>::Ok(std::move(traj));
}

bool CartesianTrajectoryPlanner::ResolveSinglePoint(
    domain::Trajectory& traj,
    int index,
    const nl::core::RbRobot& robot,
    int solution_index)
{
    if (index < 0 || index >= static_cast<int>(traj.points.size()))
        return false;

    auto& pt = traj.points[index];

    nl::utils::Q seed;
    if (index > 0 && traj.points[index - 1].status ==
        domain::TrajectoryPointStatus::kOk) {
        seed = foundation::ToQ(traj.points[index - 1].joint_state);
    } else {
        seed = foundation::ToQ(pt.joint_state);
    }

    std::vector<nl::utils::Q> solutions;
    if (!nl::kinematics::ComputeIkAllSolutions(
            robot, foundation::ToGpTrsf(pt.tcp_pose), seed, solutions)) {
        return false;
    }

    if (solution_index < 0 ||
        solution_index >= static_cast<int>(solutions.size())) {
        return false;
    }

    pt.joint_state = foundation::ToJointState(solutions[solution_index]);
    pt.status = domain::TrajectoryPointStatus::kOk;

    if (index > 0) {
        const auto& prev = traj.points[index - 1];
        if (prev.status == domain::TrajectoryPointStatus::kOk &&
            HasJointJump(
                foundation::ToQ(prev.joint_state),
                foundation::ToQ(pt.joint_state),
                30.0)) {
            pt.status = domain::TrajectoryPointStatus::kJointJump;
        }
    }

    return true;
}

} // namespace planning
```

---

### 6. `src/planning/CMakeLists.txt`（修改）

从 INTERFACE 库改为 SHARED 库，加入新源文件和依赖。

```cmake
set(TargetName SSPlanning)

set(Headers
    GrindingPlanningExport.h
    PlanningTypes.h
    ITrajectoryPlanner.h
    IKinematicsService.h
    ICollisionChecker.h
    CartesianTrajectoryPlanner.h
)

set(Sources
    CartesianTrajectoryPlanner.cpp
)

add_library(${TargetName} SHARED ${Headers} ${Sources})

target_compile_definitions(${TargetName}
    PRIVATE
    GRINDING_PLANNING_BUILDING_DLL
)

target_include_directories(${TargetName}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TargetName}
    PUBLIC
    SSFoundation
    SSDomain
    GrindingCore
    GrindingKinematics
    SSGeometry
)

set_target_properties(${TargetName} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
```

---

### 7. `src/ui/CMakeLists.txt`（修改）

删除 `TrajectoryPlanner.h/.cpp`，加 `SSPlanning` 依赖。

```cmake
set(HEADERS
    GrindingUIExport.h
    Commands.h
)

set(MOC_HEADER
    MainWindow.h
    OcctViewWidget.h
    JogPanel.h
    RobotController.h
    TrajectoryPanel.h
    TrajectoryPlayer.h
    MovementPanel.h
)

qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADER})

set(SOURCES
    MainWindow.cpp
    OcctViewWidget.cpp
    JogPanel.cpp
    RobotController.cpp
    TrajectoryPanel.cpp
    TrajectoryPlayer.cpp
    Commands.cpp
    MovementPanel.cpp
    ${MOC_SOURCES}
)

add_library(GrindingUI SHARED ${HEADERS} ${MOC_HEADER} ${SOURCES})

target_compile_definitions(GrindingUI PRIVATE GRINDING_UI_BUILDING_DLL)

target_include_directories(GrindingUI PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(GrindingUI PUBLIC
    SSFoundation
    SSDomain
    SSPlanning
    GrindingCore
    GrindingOcc
    GrindingKinematics
    Qt5::Gui
    Qt5::Widgets
    Qt5::OpenGL
)

set_target_properties(GrindingUI PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
```

---

### 8. `src/ui/MainWindow.h`（修改差异）

```cpp
// 删除：
// #include "TrajectoryPlanner.h"

// 新增：
#include "planning/CartesianTrajectoryPlanner.h"

// 成员变量（原来是 TrajectoryPlanner::Config planner_config_）改为：
planning::CartesianTrajectoryPlanner planner_;
planning::PlanningRequest planner_config_;
```

---

### 9. `src/ui/MainWindow.cpp`（修改差异）

**`OnPlanTrajectory()`**：

```cpp
void MainWindow::OnPlanTrajectory(double approach_dist)
{
    if (waypoints_.empty()) {
        QMessageBox::warning(this, tr("Warning"),
            tr("No waypoints. Please generate waypoints first."));
        return;
    }

    domain::WaypointSet ws;
    for (const auto& wp : waypoints_) {
        // 原来的 base_wps 转换逻辑保持不变
        domain::Waypoint w;
        gp_Trsf t = /* 原有坐标变换 */;
        w.pose = foundation::ToPose(t);
        ws.points.push_back(w);
    }

    planning::PlanningRequest req = planner_config_;
    req.approach_dist = approach_dist;

    auto result = planner_.Plan(
        controller_->GetRobot(),
        controller_->GetCurrentAngles(),
        ws,
        req);

    if (!result) {
        QMessageBox::warning(this, tr("Warning"),
            QString::fromStdString(result.error().message));
        return;
    }

    trajectory_ = result.value();
    traj_panel_->SetTrajectory(trajectory_);
    scene_->UpdateTrajectory(trajectory_);
}
```

**`OnIkSolutionChanged()`**：

```cpp
void MainWindow::OnIkSolutionChanged(int point_index, int solution_index)
{
    if (planner_.ResolveSinglePoint(trajectory_, point_index,
                                    controller_->GetRobot(), solution_index)) {
        traj_panel_->UpdatePoint(point_index, trajectory_.points[point_index]);
        scene_->UpdateTrajectoryPoint(point_index, trajectory_.points[point_index]);
    }
}
```

---

---

## 设计模式说明

### 1. Strategy 模式

**什么时候用**：同一件事有多种做法，且将来可能替换，就把"做法"抽成接口。

**本次怎么用**：
```
ITrajectoryPlanner          ← 接口（契约）
  └── CartesianTrajectoryPlanner  ← 具体策略（笛卡尔插值）
  └── (未来) OmplTrajectoryPlanner ← 另一种策略，调用方无需改动
```

**识别信号**：看到 `class IXxx { virtual Xxx() = 0; }` + 多个实现类，就是 Strategy。

**好处**：调用方（MainWindow / SceneAppService）只依赖接口，不依赖具体算法。
换算法不改调用方代码，符合**开闭原则**（对扩展开放，对修改封闭）。

---

### 2. Facade 模式

**什么时候用**：一组复杂的子操作需要按固定顺序执行，对外只暴露一个简单入口。

**本次怎么用**：
```
Plan()  ←  对外唯一入口
  ├── ComputeApproachPose()    步骤 1
  ├── InterpolateMoveJ()       步骤 2
  ├── InterpolateMoveL() × N   步骤 3
  └── HasJointJump() × N       步骤 4（后处理）
```

调用方只写 `planner_.Plan(...)` 一行，不需要知道内部有几步。

**识别信号**：一个方法内部调用多个私有方法完成一个完整流程。

---

### 3. Pimpl 精神（头文件隔离）

**什么时候用**：实现里必须用第三方类型（OCCT、Eigen 等），但不想把这些类型暴露在头文件里，
否则所有 include 这个头文件的地方都要依赖那个三方库。

**完整 Pimpl 写法**（当第三方类型是**成员变量**时）：
```cpp
// foo.h ── 不 include third_party.h
struct Rep;
class Foo { Rep* rep_; };

// foo.cpp ── 只有这里知道 Rep 的真实内容
#include "third_party.h"
struct Rep { ThirdParty obj; };
```

**本次的简化版**：helper 函数不是成员变量，直接把它们放进 `.cpp` 匿名 namespace，
头文件里完全不声明这些函数，效果等价。

**识别信号**：头文件里看到 `struct Rep;` 前置声明 + `.cpp` 里才定义 `struct Rep { ... }`。

---

### 4. Result 惯用法（非 GoF，现代 C++ 惯用法）

**什么时候用**：函数可能失败，需要把错误原因传递给调用方，但不想用异常。

**对比**：
```cpp
// 旧写法：失败时返回空对象，调用方不知道为什么失败
Trajectory Plan(...);

// 新写法：失败时携带错误描述，调用方被迫处理
Result<Trajectory> Plan(...);
// 调用方：
auto result = planner_.Plan(...);
if (!result) { /* 处理错误 result.error().message */ }
trajectory_ = result.value();
```

**好处**：
- 错误是显式的，不会被忽略（`if (!result)` 是强制的）
- 不依赖异常机制，性能可预测
- 函数签名本身就告诉你"这个操作可能失败"

**识别信号**：返回类型是 `Result<T>`，调用处有 `if (!result)` 检查。

---

  1. 内联标注 — 每个代码块关键位置加了 // [Strategy] / // [Facade] / // [Pimpl 精神] / // [Result 惯用法]
  注释，对应具体代码行
  2. 末尾总结节 — 4 个模式各一节，包含：
    - 什么时候用（触发条件）
    - 本次怎么用（结构图）
    - 识别信号（以后怎么认出来）
    - 好处

  以后写新功能时按这个模式套就行：接口 + 实现类 = Strategy，一个入口封多步 = Facade，三方类型下沉到 .cpp =
  Pimpl，可失败函数用 Result<T> = Result 惯用法。

## 完成标志

- [ ] `src/ui/TrajectoryPlanner.h` 和 `.cpp` 已删除
- [ ] `GrindingUI` target 不包含规划算法代码
- [ ] `SSPlanning` 编译为 SHARED 库（`SSPlanning.dll`）
- [ ] 编译通过（无 warning）
- [ ] 轨迹规划功能与 Phase 3 一致

## 验证

```bash
# 编译
cmake --build build --config Release

# 运行测试
cd build && ctest --output-on-failure -C Release

# 手动验证
# 1. 启动程序，导入机器人 + 工件
# 2. 生成 waypoints
# 3. Ctrl+P 规划轨迹
# 4. TrajectoryPanel 显示正常
# 5. 面板中切换 IK 解正常
```
