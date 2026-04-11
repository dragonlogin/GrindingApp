# GrindingApp — 项目架构地图

> 用途：减少 Claude 每次任务的探索 token。任务开始前先看此文件定位相关代码。

---

## 项目目标

Qt5 + OpenCASCADE (OCCT) 机器人磨削仿真应用。
- 加载工业机器人（URDF 格式）、工具、工件
- 实时关节 Jog + 正运动学可视化（KDL DH 矩阵）
- 轨迹规划（MoveJ/MoveL）+ 场景树管理

---

## CMake 模块架构

```
GrindingUtils (SHARED)          src/utils/
    │  纯 C++ 数据类型：Vector3d、Q
    │
SSFoundation (SHARED)           src/foundation/
    │  Result<T>、Pose、JointState、Conversions；depends on GrindingUtils + OCCT
    │
SSDomain (SHARED)               src/domain/
    │  domain::Robot（纯运动学，无渲染）；depends on SSFoundation
    │
SSModelImport (SHARED)          src/model_import/
    │  IRobotImporter / IToolImporter 接口 + URDF 实现；depends on SSDomain + GrindingCore
    │
GrindingCore (SHARED)           src/core/
    │  Qt5::Xml，RbXmlParser（过渡用，待后续 Phase 删除）；depends on GrindingUtils
    │
GrindingOcc (SHARED)            src/occ/
    │  OpenCASCADE 几何操作；depends on GrindingCore + SSFoundation
    │
GrindingKinematics (SHARED)     src/kinematics/
    │  KDL 运动学求解器；depends on SSFoundation + SSDomain
    │
SSPlanning (SHARED)             src/planning/
    │  IKinematicsService / KdlKinematicsService / ITrajectoryPlanner；depends on SSDomain + GrindingKinematics
    │
GrindingUI (SHARED)             src/ui/
    │  Qt5 UI 层；depends on SSModelImport + SSPlanning + GrindingOcc + GrindingKinematics
    │
GrindingApp (EXE)               execute/main.cpp
```

---

## 构建产物

| CMake 目标 | 类型 | 产出 |
|---|---|---|
| `GrindingUtils` | SHARED | `libGrindingUtils.dylib` |
| `SSFoundation` | SHARED | `libSSFoundation.dylib` |
| `SSDomain` | SHARED | `libSSDomain.dylib` |
| `SSModelImport` | SHARED | `libSSModelImport.dylib` |
| `GrindingCore` | SHARED | `libGrindingCore.dylib` |
| `GrindingOcc` | SHARED | `libGrindingOcc.dylib` |
| `GrindingKinematics` | SHARED | `libGrindingKinematics.dylib` |
| `SSPlanning` | SHARED | `libSSPlanning.dylib` |
| `GrindingUI` | SHARED | `libGrindingUI.dylib` |
| `GrindingApp` | EXE | `bin/GrindingApp` |
| `TestFoundtion` | Test EXE | `bin/TestFoundtion` |
| `TestPlanning` | Test EXE | `bin/TestPlanning` |
| `TestModelImport` | Test EXE | `bin/TestModelImport` |

---

## 目录结构

```
GrindingApp/
├── execute/
│   └── main.cpp                          # QApplication + MainWindow 入口
├── src/
│   ├── CMakeLists.txt                    # add_subdirectory 聚合（include src/ 全局路径）
│   ├── utils/
│   │   ├── Vector3d.h                    # nl::utils::Vector3d（全 inline）
│   │   └── Q.h/cpp                       # nl::utils::Q（关节角配置）
│   ├── foundation/
│   │   ├── Result.h                      # foundation::Result<T>（错误处理惯用法）
│   │   ├── Pose.h                        # foundation::Pose（4x4 齐次矩阵，单位 m）
│   │   ├── JointState.h                  # foundation::JointState（关节角度，单位 deg）
│   │   ├── Error.h                       # foundation::Error
│   │   ├── Conversions.h/cpp             # Pose ↔ gp_Trsf, JointState ↔ Q 转换
│   │   └── UnitTypes.h                   # 单位标注类型
│   ├── domain/
│   │   ├── Robot.h                       # domain::Robot + RobotJoint（纯运动学，无渲染）
│   │   ├── Waypoint.h                    # domain::Waypoint
│   │   ├── WaypointSet.h                 # domain::WaypointSet
│   │   └── Trajectory.h                  # domain::Trajectory
│   ├── model_import/
│   │   ├── ModelImportExport.h           # MODEL_IMPORT_EXPORT 宏
│   │   ├── RobotDefinition.h             # model_import::RobotDefinition + MeshRef
│   │   ├── ToolDefinition.h              # model_import::ToolDefinition
│   │   ├── IRobotImporter.h              # 纯虚接口：Import() → Result<RobotDefinition>
│   │   ├── IToolImporter.h               # 纯虚接口：Import() → Result<ToolDefinition>
│   │   ├── UrdfRobotImporter.h/cpp       # URDF 机器人导入器（实现 IRobotImporter）
│   │   └── UrdfToolImporter.h/cpp        # URDF 工具导入器（实现 IToolImporter）
│   ├── core/
│   │   ├── GrindingCoreExport.h          # GRINDING_CORE_EXPORT 宏
│   │   └── RbXmlParser.h/cpp             # nl::core::RbRobot（过渡结构，待删）
│   ├── occ/
│   │   ├── GrindingOccExport.h           # GRINDING_OCC_EXPORT 宏
│   │   ├── StlLoader.h/cpp               # nl::occ::StlLoader：.stl → TopoDS_Shape
│   │   ├── StepImporter.h/cpp            # nl::occ::StepImporter：.step → TopoDS_Shape
│   │   ├── RobotDisplay.h/cpp            # nl::occ：DhTrsf / RpyPosTrsf / ComputeFkHome
│   │   ├── IWaypointAlgo.h               # nl::occ::IWaypointAlgo 接口
│   │   ├── WaypointGenerator.h/cpp       # nl::occ::WaypointGenerator（Bridge）
│   │   ├── WaypointGridAlgo.h/cpp        # UV 网格采样算法
│   │   ├── WaypointPlanarAlgo.h/cpp      # 平面切割采样算法
│   │   └── SurfaceWaypointGen.h/cpp      # 旧版便捷函数（待清理）
│   ├── kinematics/
│   │   ├── GrindingKinematicsExport.h    # GRINDING_KINEMATICS_EXPORT 宏
│   │   ├── IKinematicsSolver.h           # nl::kinematics::IKinematicsSolver（抽象接口）
│   │   ├── KdlSolver.h/cpp               # nl::kinematics::KdlSolver（KDL FK/IK）
│   │   ├── KdlChainBuilder.h/cpp         # domain::Robot → KDL 链构建
│   │   └── EigenSolver.h/cpp             # nl::kinematics::EigenSolver（委托 KdlSolver）
│   ├── planning/
│   │   ├── IKinematicsService.h          # planning::IKinematicsService（FK/IK 服务接口）
│   │   ├── KdlKinematicsService.h/cpp    # KDL 实现，零三方头依赖
│   │   ├── ITrajectoryPlanner.h          # planning::ITrajectoryPlanner（轨迹规划接口）
│   │   └── CartesianTrajectoryPlanner.h/cpp # MoveL 笛卡尔插值规划
│   └── ui/
│       ├── GrindingUIExport.h            # GRINDING_UI_EXPORT 宏
│       ├── MainWindow.h/cpp              # nl::ui::MainWindow：菜单、场景树、信号编排
│       ├── OcctViewWidget.h/cpp          # nl::ui::OcctViewWidget：Qt 封装 OCCT V3d_View
│       ├── RobotController.h/cpp         # nl::ui::RobotController：加载 + 渲染 + FK/IK
│       ├── JogPanel.h/cpp                # nl::ui::JogPanel：关节 Jog 滑块
│       ├── TrajectoryPanel.h/cpp         # nl::ui::TrajectoryPanel：轨迹编辑表格 Dock
│       ├── TrajectoryPlayer.h/cpp        # nl::ui::TrajectoryPlayer：回放控制 Dock
│       └── MovementPanel.h/cpp           # nl::ui::MovementPanel：6-DOF 移动面板
├── tests/
│   ├── CMakeLists.txt
│   ├── test_foundation/
│   │   └── TestFoundation.cpp            # SSFoundation 单元测试
│   ├── test_planning/
│   │   └── TestKdlKinematicsService.cpp  # KdlKinematicsService 单元测试
│   └── test_model_import/
│       └── TestUrdfRobotImporter.cpp     # UrdfRobotImporter 单元测试
├── model/
│   ├── robot/IRB140/IRB140.urdf          # ABB IRB140 URDF + STL 网格
│   └── robot/irb2400/irb2400.urdf        # IRB2400 URDF
├── docs/
│   └── superpowers/specs/                # Phase 设计文档
├── CMakeLists.txt
└── vcpkg.json
```

---

## 模块职责

### `src/foundation/` — SSFoundation

| 文件 | 职责 |
|---|---|
| `Result.h` | `Result<T>` 错误处理惯用法，`[[nodiscard]]` |
| `Pose.h` | `Pose`：4x4 齐次变换矩阵，行主序，单位 m |
| `JointState.h` | `JointState`：关节角度集合，单位 deg |
| `Conversions.h/cpp` | `ToPose(gp_Trsf)` / `ToGpTrsf(Pose)` / `ToJointState(Q)` / `ToQ(JointState)` |

### `src/domain/` — SSDomain

| 文件 | 职责 |
|---|---|
| `Robot.h` | `domain::Robot`（name, source_path, joints[]）；`RobotJoint`（DH 参数）；**纯运动学，不含渲染数据** |

### `src/model_import/` — SSModelImport

| 文件 | 职责 |
|---|---|
| `RobotDefinition.h` | `RobotDefinition = {domain::Robot model, vector<MeshRef> meshes}` |
| `ToolDefinition.h` | `ToolDefinition`：mesh 路径 + base/tcp 变换参数 |
| `IRobotImporter.h` | `IRobotImporter::Import(path)` → `Result<RobotDefinition>` |
| `IToolImporter.h` | `IToolImporter::Import(path)` → `Result<ToolDefinition>` |
| `UrdfRobotImporter.h/cpp` | 实现 `IRobotImporter`；内部调 `RbXmlParser::Parse()` 后转换 |
| `UrdfToolImporter.h/cpp` | 实现 `IToolImporter`；Qt XML 直接解析工具 URDF |

### `src/core/` — GrindingCore（过渡层，待后续 Phase 删除）

| 文件 | 职责 |
|---|---|
| `RbXmlParser.h/cpp` | `RbXmlParser::Parse(path)` → `RbRobot`；目前仍被 `UrdfRobotImporter` 调用 |

### `src/kinematics/` — GrindingKinematics

| 文件 | 职责 |
|---|---|
| `IKinematicsSolver.h` | `IKinematicsSolver`：`ComputeFk` / `ComputeIk` 抽象接口 |
| `KdlSolver.h/cpp` | KDL 正逆运动学，参数 `domain::Robot` + `Q` |
| `KdlChainBuilder.h/cpp` | `domain::Robot` → `KDL::Chain` 构建 |
| `EigenSolver.h/cpp` | Eigen3 实现（委托给 `KdlSolver`） |

### `src/planning/` — SSPlanning

| 文件 | 职责 |
|---|---|
| `IKinematicsService.h` | `ComputeFk` / `ComputeIk` / `ComputeIkAll` 服务门面接口 |
| `KdlKinematicsService.h/cpp` | KDL 实现；头文件零三方依赖（Pimpl 精神） |
| `ITrajectoryPlanner.h` | 轨迹规划接口，持有 `IKinematicsService&` |
| `CartesianTrajectoryPlanner.h/cpp` | MoveL 笛卡尔插值 |

### `src/ui/` — GrindingUI

| 文件 | 职责 |
|---|---|
| `RobotController.h/cpp` | 注入 `IRobotImporter` + `IToolImporter`；持有 `domain::Robot`；管理 AIS 渲染和 FK 更新 |
| `MainWindow.h/cpp` | UI 编排：信号连接、场景树、构造并注入 `UrdfRobotImporter`/`UrdfToolImporter` |

---

## 核心数据结构

```cpp
// domain::Robot（src/domain/Robot.h）
struct RobotJoint { string name; double alpha_deg, a_mm, d_mm, offset_deg; };
struct Robot      { string name, source_path; vector<RobotJoint> joints; };

// model_import::RobotDefinition（src/model_import/RobotDefinition.h）
struct MeshRef        { string name, ref_joint, mesh_file; Vector3d rpy, pos; };
struct RobotDefinition { domain::Robot model; vector<MeshRef> meshes; };

// model_import::ToolDefinition（src/model_import/ToolDefinition.h）
struct ToolDefinition {
    string name, source_path, mesh_file;
    Vector3d mesh_scale, base_rpy_deg, base_pos_mm, tcp_rpy_deg, tcp_pos_mm;
};

// foundation::Result<T>（src/foundation/Result.h）
Result<T>::Ok(value) / Result<T>::Fail(error)
bool ok(); T& value(); Error& error();
```

---

## 数据流（Robot 导入）

```
IRB140.urdf
    │
    ▼  UrdfRobotImporter::Import()
    │      └─ RbXmlParser::Parse() → RbRobot → ToRobotDefinition()
    ▼
RobotDefinition { model: domain::Robot, meshes: vector<MeshRef> }
    │
    ├──► model → RobotController::current_robot_ (domain::Robot)
    │       └──► KdlSolver::ComputeFk() → gp_Trsf × N
    │
    └──► meshes → StlLoader::Load() → AIS_Shape × N → OcctViewWidget
```

---

## 任务 → 文件速查表

| 任务 | 主要文件 |
|---|---|
| 加载新格式机器人 | 实现 `IRobotImporter`，在 `MainWindow` 构造时注入 |
| 修改 URDF 解析逻辑 | `src/model_import/UrdfRobotImporter.cpp` |
| 修改工具加载逻辑 | `src/model_import/UrdfToolImporter.cpp` |
| 修改 FK/IK 逻辑 | `src/kinematics/KdlSolver.cpp` |
| 替换运动学后端 | 实现 `IKinematicsService`，在 `MainWindow` 中替换 `kin_service_` |
| 修改 Jog 面板 UI | `src/ui/MainWindow.cpp` → `SetupJogPanel()` |
| 修改场景树 | `src/ui/MainWindow.cpp` → `AddRobot()` / `AddTool()` |
| 修改工件加载 | `src/ui/MainWindow.cpp` → `OnImportWorkpiece()` + `src/occ/StepImporter.h/cpp` |
| 修改 3D 视口交互 | `src/ui/OcctViewWidget.cpp` |
| 修改路径生成算法 | `src/occ/WaypointGridAlgo.cpp` 或 `WaypointPlanarAlgo.cpp` |
| 修改轨迹规划逻辑 | `src/planning/CartesianTrajectoryPlanner.cpp` |
| 修改轨迹回放 | `src/ui/TrajectoryPlayer.cpp` |
| 添加新测试 | `tests/test_XXX/` 下新建 + 对应 `CMakeLists.txt` |

---

## 运动学约定

- **DH 约定**：Craig 1989（`Rz(θ) * Tz(d) * Tx(a) * Rx(α)`）
- **单位**：长度 mm（RbRobot/MeshRef），m（Pose/gp_Trsf），角度 deg（Q/JointState）
- **关节顺序**：Joint1..Joint6，Joint6 = TCP
- **RPY 顺序**：`Rz(yaw) * Ry(pitch) * Rx(roll)`，MeshRef.rpy = `{yaw, pitch, roll}`

---

## 构建与测试

```bash
# 构建
cmake --build build --config Release

# 测试
cd build && ctest --output-on-failure -C Release
```
