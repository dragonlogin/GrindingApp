# Phase 5 详细设计文档：抽出 Kinematics 服务门面

**日期**：2026-04-03  
**阶段**：Phase 5 / 7  
**状态**：设计已确认，待实现

---

## 目标

把自由函数风格的 FK/IK 运动学能力整理成稳定的服务接口，为 `planning/` 层以上提供一致的调用方式，彻底隐藏 KDL / OCCT / Eigen 等底层库细节。

---

## 设计决策

| 决策点 | 选择 | 原因 |
|---|---|---|
| `IKinematicsService` Robot 参数 | `domain::Robot` | **[Strategy]** 接口直接用干净类型，不带旧依赖 |
| `domain::Robot` 是否含 joint limits | 不含 | limits 由 `KdlKinematicsService` 内部从 URDF 读 |
| `domain::Robot` 身份字段 | 含 `name` + `source_path` | `source_path` 是读 URDF 限位的定位依据 |
| `KdlChainBuilder` 适配 | 全面迁移 `domain::Robot`，废弃旧接口 | **[Strategy]** 统一依赖方向，不留双轨 |
| `CartesianTrajectoryPlanner` | 同步换 `domain::Robot` + `JointState` | planning 层一步到位，不留过渡残留 |
| 旧 `RobotKinematics` 自由函数 | 直接删除 | 无外部调用方，保留无意义 |
| `IKinematicsService` 注入方式 | 构造函数注入到 `ITrajectoryPlanner` 基类 | **[Strategy]** 依赖倒置，所有子类共用，可注入 mock |
| 类型转换 | 复用 `foundation::Conversions.h` | `ToPose` / `ToGpTrsf` / `ToJointState` / `ToQ` 已实现 |

---

## 涉及文件一览

### 新增

| 文件 | 说明 |
|---|---|
| `src/planning/KdlKinematicsService.h` | `IKinematicsService` 的 KDL 实现，头文件零三方依赖 |
| `src/planning/KdlKinematicsService.cpp` | 内部使用 KDL / OCCT，通过 `foundation::Conversions` 转换 |
| `tests/TestKdlKinematicsService.cpp` | 单元测试，替代即将删除的 `TestRobotKinematics.cpp` |

### 修改

| 文件 | 变化 |
|---|---|
| `src/domain/Robot.h` | 填充完整结构（见下节） |
| `src/planning/IKinematicsService.h` | 填充完整接口定义 |
| `src/planning/ITrajectoryPlanner.h` | 添加构造函数 + `kin_` 成员 |
| `src/planning/CartesianTrajectoryPlanner.h/cpp` | Robot/JointState 类型替换，调用改走 `kin_` |
| `src/kinematics/KdlChainBuilder.h/cpp` | `RbRobot` → `domain::Robot`，include 替换 |
| `src/kinematics/CMakeLists.txt` | 移除 `RobotKinematics.h/cpp` |
| `src/planning/CMakeLists.txt` | 添加 `KdlKinematicsService.h/cpp` |
| `tests/CMakeLists.txt` | 添加 `TestKdlKinematicsService`，移除 `TestRobotKinematics` |

### 删除

| 文件 | 原因 |
|---|---|
| `src/kinematics/RobotKinematics.h` | 自由函数全部由 `KdlKinematicsService` 取代 |
| `src/kinematics/RobotKinematics.cpp` | 同上 |
| `tests/TestRobotKinematics.cpp` | 用例迁移到 `TestKdlKinematicsService.cpp` |

---

## 详细设计

### 1. `domain::Robot`

```cpp
// src/domain/Robot.h
#pragma once

#include <string>
#include <vector>

namespace domain {

struct RobotJoint {
    std::string name;
    double alpha_deg;
    double a_mm;       // 连杆长度，毫米
    double d_mm;       // 连杆偏置，毫米
    double offset_deg; // home 位置偏移
};

// [Value Object] 纯运动学领域模型，不含渲染/网格数据
struct Robot {
    std::string name;
    std::string source_path;  // URDF 路径，KdlKinematicsService 用于读关节限位
    std::vector<RobotJoint> joints;
};

} // namespace domain
```

**不包含**：`drawables`（网格显示）属于渲染层，由 `SSGeometry` / `SSQtDesktop` 管理。

---

### 2. `IKinematicsService`

```cpp
// src/planning/IKinematicsService.h
#pragma once

#include <vector>
#include "foundation/Pose.h"
#include "foundation/JointState.h"
#include "foundation/Result.h"
#include "domain/Robot.h"

namespace planning {

// [Strategy] 运动学抽象接口，隔离底层求解器实现
class IKinematicsService {
public:
    virtual ~IKinematicsService() = default;

    // 正向运动学：返回每个关节的世界坐标变换（index 0 = Joint1）
    virtual foundation::Result<std::vector<foundation::Pose>> ComputeFk(
        const domain::Robot& robot,
        const foundation::JointState& joints) = 0;

    // 逆向运动学：返回最优单解（关节空间距离 seed 最近）
    virtual foundation::Result<foundation::JointState> ComputeIk(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) = 0;

    // 逆向运动学：返回所有解（最多 8 个，按距 seed 距离排序）
    virtual foundation::Result<std::vector<foundation::JointState>> ComputeIkAll(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) = 0;
};

} // namespace planning
```

---

### 3. `KdlKinematicsService`

```cpp
// src/planning/KdlKinematicsService.h
#pragma once

#include "GrindingPlanningExport.h"
#include "IKinematicsService.h"

namespace planning {

// [Strategy] IKinematicsService 的 KDL 实现
// [Pimpl 精神] KDL/OCCT 类型全部在 .cpp 中，头文件零三方依赖
class GRINDING_PLANNING_EXPORT KdlKinematicsService : public IKinematicsService {
public:
    foundation::Result<std::vector<foundation::Pose>> ComputeFk(
        const domain::Robot& robot,
        const foundation::JointState& joints) override;

    foundation::Result<foundation::JointState> ComputeIk(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) override;

    foundation::Result<std::vector<foundation::JointState>> ComputeIkAll(
        const domain::Robot& robot,
        const foundation::Pose& target,
        const foundation::JointState& seed) override;
};

} // namespace planning
```

**`.cpp` 结构：**

```cpp
// src/planning/KdlKinematicsService.cpp
// 三方库只在 .cpp 中 include
#include <gp_Trsf.hxx>
#include <kdl/chain.hpp>
#include "kinematics/KdlChainBuilder.h"
#include "kinematics/KdlSolver.h"
#include "foundation/Conversions.h"  // ToPose / ToGpTrsf / ToJointState / ToQ

// ComputeIk 内部调用 ComputeIkAll 取第 0 个解，不重复实现
// 关节限位通过 BuildKdlJointLimits(robot, ...) 读取（robot.source_path → URDF）
```

---

### 4. `ITrajectoryPlanner` + `CartesianTrajectoryPlanner`

```cpp
// src/planning/ITrajectoryPlanner.h
class ITrajectoryPlanner {
public:
    // [Strategy] 基类持有运动学服务，所有子类共用
    explicit ITrajectoryPlanner(IKinematicsService& kin) : kin_(kin) {}
    virtual ~ITrajectoryPlanner() = default;

    virtual foundation::Result<domain::Trajectory> Plan(
        const domain::Robot& robot,
        const foundation::JointState& current,
        const domain::WaypointSet& waypoints,
        const PlanningRequest& request) = 0;

protected:
    IKinematicsService& kin_;
};
```

```cpp
// src/planning/CartesianTrajectoryPlanner.h
class GRINDING_PLANNING_EXPORT CartesianTrajectoryPlanner : public ITrajectoryPlanner {
public:
    explicit CartesianTrajectoryPlanner(IKinematicsService& kin)
        : ITrajectoryPlanner(kin) {}

    foundation::Result<domain::Trajectory> Plan(
        const domain::Robot& robot,
        const foundation::JointState& current,
        const domain::WaypointSet& waypoints,
        const PlanningRequest& request) override;

    // UI 交互用：从已有轨迹中重新选择某点的 IK 解
    bool ResolveSinglePoint(
        domain::Trajectory& traj,
        int index,
        const domain::Robot& robot,
        int solution_index);
};
```

**`.cpp` 内调用变化：**

```cpp
// 旧：bool ok = ComputeIk(robot, target_trsf, seed_q, out_q);
// 新：
auto result = kin_.ComputeIk(robot, foundation::ToPose(target_trsf), current);
if (!result) return foundation::Result<domain::Trajectory>::Fail(result.error());
foundation::JointState solution = result.value();
```

---

### 5. `KdlChainBuilder` 接口更新

```cpp
// src/kinematics/KdlChainBuilder.h
#include "domain/Robot.h"  // 替换原 RbXmlParser.h

namespace nl {
namespace kinematics {

// 从 domain::Robot 构建 KDL 链（替换旧 RbRobot 版本）
GRINDING_KINEMATICS_EXPORT KDL::Chain BuildKdlChain(
    const domain::Robot& robot,
    bool include_tool_mount = false);

GRINDING_KINEMATICS_EXPORT bool BuildKdlJointLimits(
    const domain::Robot& robot,  // 内部从 robot.source_path 读 URDF
    KDL::JntArray& q_min,
    KDL::JntArray& q_max);

// 保留：独立从 URDF 路径构建，不依赖 domain::Robot
GRINDING_KINEMATICS_EXPORT KDL::Chain BuildKdlChainFromUrdfFile(
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& tip_link);

GRINDING_KINEMATICS_EXPORT bool BuildKdlJointLimitsFromUrdfFile(
    const std::string& urdf_path,
    const std::string& base_link,
    const std::string& tip_link,
    KDL::JntArray& q_min,
    KDL::JntArray& q_max);

} // namespace kinematics
} // namespace nl
```

**`.cpp` 变化**：字段名适配（`joint.a` → `joint.a_mm`，`joint.d` → `joint.d_mm`）。

---

## 测试设计

```cpp
// tests/TestKdlKinematicsService.cpp
class TestKdlKinematicsService : public QObject {
    Q_OBJECT
private slots:
    // 正常值：FK 返回非单位阵的有效位姿
    void testComputeFk_returnsValidPose();

    // Round-trip：FK(q) → IK(pose, seed=q) → FK(q2)，末端位姿误差 < 0.01mm
    void testFkIkRoundTrip();

    // 边界值：空 joints（size=0）返回 Fail，不崩溃
    void testComputeFk_emptyJoints_returnsFail();

    // 多解：ComputeIkAll 返回 ≥ 1 个解，所有解 FK 后末端位姿一致
    void testComputeIkAll_multipleSolutions();

    // 退化：目标超出工作空间，ComputeIk 返回 Fail
    void testComputeIk_unreachableTarget_returnsFail();
};
```

`TestRobotKinematics.cpp` 中的 Round-trip 和精度验证用例迁移至此，原文件删除。

---

## 依赖关系图

```
foundation::Conversions
    ↓
KdlKinematicsService (.cpp)
    ├── kinematics::KdlChainBuilder   (domain::Robot)
    ├── kinematics::KdlSolver
    └── foundation::Conversions       (类型转换)

IKinematicsService (planning/)
    ↑ 实现
KdlKinematicsService

ITrajectoryPlanner (持有 IKinematicsService&)
    ↑ 继承
CartesianTrajectoryPlanner
```

**禁止的依赖**：`domain/` 不能 include KDL / OCCT / Qt。`planning/` 头文件不能暴露三方类型。

---

## 完成标志

- [ ] `domain::Robot` 结构体已填充
- [ ] `IKinematicsService` 接口已填充
- [ ] `KdlKinematicsService` 实现完成，编译通过
- [ ] `KdlChainBuilder` 全面迁移 `domain::Robot`
- [ ] `CartesianTrajectoryPlanner` 签名和内部调用全部更新
- [ ] `RobotKinematics.h/cpp` 已删除
- [ ] `TestKdlKinematicsService` 全部用例通过
- [ ] 轨迹规划功能端到端不回归

---

## 设计模式说明

### Strategy

**什么时候用**：需要在运行时替换算法实现，调用方不依赖具体类。

**本次怎么用**：`IKinematicsService` 定义 FK/IK 契约，`KdlKinematicsService` 是当前实现。测试时可注入 mock 实现；未来可替换为 Eigen 或其他求解器。

**好处**：`CartesianTrajectoryPlanner` 与 KDL 完全解耦，单元测试无需真实机器人模型。

**缺点/注意**：多一层虚函数调用（运动学场景下可忽略）。

**识别信号**：`class IXxx { virtual Xxx() = 0; }` + 构造函数注入。

### Pimpl 精神（匿名 namespace）

**什么时候用**：头文件需要对上层零三方依赖，但实现必须用到三方库。

**本次怎么用**：`KdlKinematicsService.h` 不 include 任何 KDL/OCCT 头，所有三方 include 和辅助函数在 `.cpp` 中。

**好处**：`planning/` 以上的层编译不需要 KDL/OCCT 头文件路径。

**缺点/注意**：只适合自由函数风格的 helper；若需要在成员函数间共享 KDL 状态，需升级为真正的 Pimpl。

**识别信号**：头文件极简，`.cpp` 顶部有大段三方 include + 匿名 namespace。
