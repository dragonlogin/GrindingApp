# GrindingApp — 项目架构地图

> 用途：减少 Claude 每次任务的探索 token。任务开始前先看此文件定位相关代码。

---

## 项目目标

Qt5 + OpenCASCADE (OCCT) 机器人磨削仿真应用。
- 加载工业机器人（RobWork `.rb.xml` 格式）、工具、工件
- 实时关节 Jog + 正运动学可视化（Eigen3 DH 矩阵）
- 场景树管理（Station Manager）

---

## CMake 模块架构

```
GrindingUtils (SHARED DLL)      src/utils/
    │  纯 C++ 数据类型：Vector3d、Q
    │
GrindingCore  (SHARED DLL)      src/core/
    │  Qt5::Core/Xml，共享数据类型；depends on GrindingUtils
    │
    ├── GrindingOcc  (SHARED DLL)   src/occ/
    │     OpenCASCADE OCCT libs
    │
    └── GrindingKinematics (SHARED DLL)  src/kinematics/
          Eigen3

GrindingUI  (SHARED DLL)        src/ui/
    Qt5::Widgets/OpenGL
    └── 依赖 GrindingCore + GrindingOcc + GrindingKinematics

GrindingApp  (EXE)   execute/main.cpp  → links all four DLLs
TestRobotKinematics  (EXE)  tests/     → links GrindingOcc + GrindingKinematics + Qt5::Test
```

PUBLIC include 目录随依赖链自动传播，无需消费方手动添加：

| 模块 | namespace | 暴露的 include 路径 |
|---|---|---|
| `GrindingUtils` | `nl::utils` | `src/utils/` |
| `GrindingCore` | `nl::core` | `src/core/`（继承 `src/utils/`） |
| `GrindingOcc` | `nl::occ` | `src/occ/`（继承 `src/core/`） |
| `GrindingKinematics` | `nl::kinematics` | `src/kinematics/`（继承 `src/core/`） |
| `GrindingUI` | `nl::ui` | `src/ui/`（继承所有上游） |

---

## 构建产物

| CMake 目标 | 类型 | 输出路径 |
|---|---|---|
| `GrindingUtils` | SHARED DLL | `build/bin/Release/GrindingUtils.dll` |
| `GrindingCore` | SHARED DLL | `build/bin/Release/GrindingCore.dll` |
| `GrindingOcc` | SHARED DLL | `build/bin/Release/GrindingOcc.dll` |
| `GrindingKinematics` | SHARED DLL | `build/bin/Release/GrindingKinematics.dll` |
| `GrindingUI` | SHARED DLL | `build/bin/Release/GrindingUI.dll` |
| `GrindingApp` | Executable | `build/bin/Release/GrindingApp.exe` |
| `TestRobotKinematics` | Test Executable | `build/tests/Release/TestRobotKinematics.exe` |

---

## 目录结构

```
GrindingApp/
├── execute/
│   └── main.cpp                       # 可执行入口：QApplication + MainWindow
├── src/
│   ├── CMakeLists.txt                 # add_subdirectory 聚合
│   ├── utils/
│   │   ├── CMakeLists.txt             # GrindingUtils (SHARED DLL)
│   │   ├── GrindingUtilsExport.h      # GRINDING_UTILS_EXPORT 宏
│   │   ├── Vector3d.h                 # nl::utils::Vector3d（全 inline，无 .cpp）
│   │   └── Q.h/cpp                    # nl::utils::Q（关节角配置，封装 double[]）
│   ├── core/
│   │   ├── CMakeLists.txt             # GrindingCore (SHARED DLL)
│   │   ├── GrindingCoreExport.h       # GRINDING_CORE_EXPORT 宏
│   │   └── RbXmlParser.h/cpp          # nl::core：RbRobot / RbJoint / RbDrawable
│   ├── ui/
│   │   ├── CMakeLists.txt             # GrindingUI (SHARED DLL)
│   │   ├── GrindingUIExport.h         # GRINDING_UI_EXPORT 宏
│   │   ├── MainWindow.h/cpp           # nl::ui::MainWindow：菜单、Jog面板、场景树
│   │   ├── OcctViewWidget.h/cpp       # nl::ui::OcctViewWidget：Qt 封装 OCCT V3d_View
│   │   ├── TrajectoryPlanner.h/cpp    # nl::ui::TrajectoryPlanner（MoveJ/MoveL + IK 规划）
│   │   ├── TrajectoryPanel.h/cpp      # nl::ui::TrajectoryPanel（轨迹编辑表格 Dock）
│   │   ├── TrajectoryPlayer.h/cpp     # nl::ui::TrajectoryPlayer（回放控制 Dock）
│   │   └── MovementPanel.h/cpp        # nl::ui::MovementPanel（6-DOF 移动面板 Dock）
│   ├── occ/
│   │   ├── CMakeLists.txt             # GrindingOcc (SHARED DLL)
│   │   ├── GrindingOccExport.h        # GRINDING_OCC_EXPORT 宏
│   │   ├── StlLoader.h/cpp            # nl::occ::StlLoader：.stl → TopoDS_Shape
│   │   ├── StepImporter.h/cpp         # nl::occ::StepImporter：.step → TopoDS_Shape
│   │   ├── RobotDisplay.h/cpp         # nl::occ：DhTrsf / RpyPosTrsf / ComputeFkHome
│   │   ├── Waypoint.h                 # nl::occ::Waypoint（路径点数据结构，全 inline）
│   │   ├── IWaypointAlgo.h            # nl::occ::IWaypointAlgo 接口 + WaypointConfig
│   │   ├── WaypointGenerator.h/cpp    # nl::occ::WaypointGenerator（Bridge 抽象层）
│   │   ├── WaypointGridAlgo.h/cpp     # nl::occ::WaypointGridAlgo（UV 网格采样）
│   │   ├── WaypointPlanarAlgo.h/cpp   # nl::occ::WaypointPlanarAlgo（平面切割采样）
│   │   ├── Trajectory.h               # nl::occ::Trajectory / TrajectoryPoint（轨迹数据结构）
│   │   └── SurfaceWaypointGen.h/cpp   # nl::occ：LargestFace / GenerateGridWaypoints（旧版便捷函数）
│   └── kinematics/
│       ├── CMakeLists.txt             # GrindingKinematics (SHARED DLL)
│       ├── GrindingKinematicsExport.h # GRINDING_KINEMATICS_EXPORT 宏
│       ├── IKinematicsSolver.h        # nl::kinematics::IKinematicsSolver（抽象接口）
│       ├── EigenSolver.h/cpp          # nl::kinematics::EigenSolver（Eigen3 实现）
│       └── RobotKinematics.h/cpp      # nl::kinematics：ComputeFk / ComputeIk（自由函数）
├── tests/
│   ├── CMakeLists.txt                 # TestRobotKinematics → links GrindingOcc + GrindingKinematics
│   └── TestRobotKinematics.cpp        # Qt Test：Eigen FK vs 手工 DH 对比
├── model/
│   ├── robot/IRB140/                  # ABB IRB140 机器人模型
│   │   ├── IRB140.rb.xml              # RobWork 格式：DH参数 + 网格路径
│   │   └── Geometry/*.stl             # 7个 STL（BASE + LINK1..6）
│   └── tool/Burr/                     # 打磨工具
│       ├── Burr.xml                   # 工具坐标系定义（TCP0 偏移）
│       ├── Burr.tool                  # 工具元数据（引用 Burr.xml）
│       └── Tool.stl                   # 工具网格
├── resources/resources.qrc            # Qt 资源（图标等）
├── translations/zh_CN.ts              # 中文翻译源文件
├── CMakeLists.txt                     # 根构建配置（聚合各模块）
└── vcpkg.json                         # vcpkg 依赖声明
```

---

## 模块职责

### `src/utils/` — GrindingUtils（基础数据类型）

| 文件 | 职责 |
|---|---|
| `Vector3d.h` | `nl::utils::Vector3d`：3D 向量，接口边界用（全 inline，无 DLL 符号） |
| `Q.h/cpp` | `nl::utils::Q`：关节角配置（封装 `std::vector<double>`，`operator[]`） |

### `src/core/` — GrindingCore（共享数据类型）

| 文件 | 职责 |
|---|---|
| `GrindingCoreExport.h` | `GRINDING_CORE_EXPORT` 宏 |
| `RbXmlParser.h/cpp` | 静态 `Parse(xml_path)` → `nl::core::RbRobot`（含 DH 参数和 STL 路径） |

### `src/ui/` — GrindingUI（Qt UI 层）

| 文件 | 职责 |
|---|---|
| `MainWindow.h/cpp` | `nl::ui::MainWindow`：UI 组装、场景树、Jog 面板、机器人/工具/工件加载 |
| `OcctViewWidget.h/cpp` | `nl::ui::OcctViewWidget`：封装 OCCT 3D 视口，暴露 `Context()` / `View()` |
| `TrajectoryPlanner.h/cpp` | `nl::ui::TrajectoryPlanner`：MoveJ/MoveL 插值 + IK 求解 + 异常检测 |
| `TrajectoryPanel.h/cpp` | `nl::ui::TrajectoryPanel`：右侧 Dock 轨迹编辑表格 + IK 选解 |
| `TrajectoryPlayer.h/cpp` | `nl::ui::TrajectoryPlayer`：底部 Dock 回放控制（播放/暂停/停止/进度/速度） |
| `MovementPanel.h/cpp` | `nl::ui::MovementPanel`：临时 6-DOF 移动面板（robot base / workpiece 共用） |

### `src/occ/` — GrindingOcc（OpenCASCADE 操作层）

| 文件 | 职责 |
|---|---|
| `StlLoader.h/cpp` | `nl::occ::StlLoader`：静态 `Load(path)` → `TopoDS_Shape` |
| `StepImporter.h/cpp` | `nl::occ::StepImporter`：静态 `Load(path, face_count*)` → `TopoDS_Shape` |
| `RobotDisplay.h/cpp` | `nl::occ`：`DhTrsf()`、`RpyPosTrsf(Vector3d)`、`ComputeFkHome()` |
| `Waypoint.h` | `nl::occ::Waypoint`：路径点（`gp_Trsf pose` + `speed_ratio`） |
| `IWaypointAlgo.h` | `nl::occ::IWaypointAlgo`：路径生成算法接口 + `WaypointConfig` |
| `WaypointGenerator.h/cpp` | `nl::occ::WaypointGenerator`：Bridge 模式抽象层（SetFace + SetAlgorithm + Generate） |
| `WaypointGridAlgo.h/cpp` | `nl::occ::WaypointGridAlgo`：UV 参数网格采样（蛇形遍历） |
| `WaypointPlanarAlgo.h/cpp` | `nl::occ::WaypointPlanarAlgo`：平面切割采样（BRepAlgoAPI_Section） |
| `Trajectory.h` | `nl::occ::Trajectory` / `TrajectoryPoint`：轨迹数据结构（MoveType + Status + joint_angles） |
| `SurfaceWaypointGen.h/cpp` | `nl::occ`：`LargestFace()` / `GenerateGridWaypoints()`（旧版便捷函数） |

### `src/kinematics/` — GrindingKinematics（运动学层）

| 文件 | 职责 |
|---|---|
| `IKinematicsSolver.h` | `nl::kinematics::IKinematicsSolver`：FK/IK 抽象接口（纯虚） |
| `EigenSolver.h/cpp` | `nl::kinematics::EigenSolver`：Eigen3 实现 FK + IK（Jacobian 伪逆 SVD） |
| `RobotKinematics.h/cpp` | `nl::kinematics`：`ComputeFk(robot, Q)` / `ComputeIk(robot, T, Q, Q&)`（自由函数，委托 EigenSolver） |

---

## 核心数据结构

```cpp
// src/utils/Vector3d.h  (nl::utils)
struct Vector3d {
    double x = 0.0, y = 0.0, z = 0.0;
    double  operator[](int i) const;
    double& operator[](int i);
};

// src/utils/Q.h  (nl::utils)
class Q {
    std::vector<double> values_;  // 关节角（度）
public:
    explicit Q(int n = 6, double val = 0.0);
    Q(std::initializer_list<double> vals);
    int    size() const;
    double  operator[](int i) const;
    double& operator[](int i);
    const double* data() const;
};

// src/core/RbXmlParser.h  (nl::core)
struct RbJoint {
    std::string name;
    double alpha_deg, a, d, offset_deg;  // Craig DH 参数（a/d 单位 mm）
};

struct RbDrawable {
    std::string name;
    std::string ref_joint;           // "Robot_Base" | "Joint1".."Joint6"
    utils::Vector3d rpy;             // 欧拉角（RobWork 顺序：Rz*Ry*Rx），单位 deg
    utils::Vector3d pos;             // 平移，单位 mm
    std::string mesh_file;           // .stl 绝对路径
};

struct RbRobot {
    std::string name;
    std::vector<RbJoint>    joints;     // 6个关节
    std::vector<RbDrawable> drawables;  // 7个网格（base + link1-6）
};
```

---

## 数据流

```
IRB140.rb.xml
    │
    ▼
RbXmlParser::Parse()          → RbRobot（DH参数 + STL路径）
    │
    ├──► StlLoader::Load()    → TopoDS_Shape × 7
    │       └──► AIS_Shape    → 显示到 OcctViewWidget
    │
    ├──► ComputeFk()          → gp_Trsf × 6（关节世界坐标）
    │       └──► AIS_Trihedron → 坐标系渲染（base/tcp 默认显示）
    │
    └──► Jog Panel (QSlider)  → joint_angles_[] → UpdateRobotDisplay()
             └──► ComputeFk() → 实时更新网格位置和坐标系
```

---

## 任务 → 文件速查表

| 任务 | 主要文件 |
|---|---|
| 修改 FK 计算逻辑 | `src/kinematics/RobotKinematics.cpp` (或 `EigenSolver.cpp`) |
| 修改 IK 逻辑 | `src/kinematics/EigenSolver.cpp` → `ComputeIk()` |
| 替换运动学后端 | 实现 `IKinematicsSolver` 接口，在 `RobotKinematics.cpp` 中切换 |
| 修改 Jog 面板 UI | `src/ui/MainWindow.cpp` → `SetupJogPanel()` |
| 修改关节坐标系渲染 | `src/ui/MainWindow.cpp` → `UpdateCoordinateFrames()` |
| 修改场景树 | `src/ui/MainWindow.cpp` → `SetupSceneTree()`, `AddRobot()`, `AddTool()` |
| 加载新格式机器人 | `src/RbXmlParser.h/cpp` |
| 修改工具加载 | `src/ui/MainWindow.cpp` → `OnLoadTool()` |
| 修改工件加载 | `src/ui/MainWindow.cpp` → `OnImportWorkpiece()` + `src/occ/StepImporter.h/cpp` |
| 修改 3D 视口交互 | `src/ui/OcctViewWidget.cpp` |
| 修改 STL 加载逻辑 | `src/occ/StlLoader.h/cpp` |
| 添加新测试 | `tests/TestRobotKinematics.cpp` + `tests/CMakeLists.txt` |
| 修改路径生成算法 | `src/occ/WaypointGridAlgo.cpp` 或 `WaypointPlanarAlgo.cpp` |
| 新增路径生成算法 | 实现 `IWaypointAlgo` 接口，在 `MainWindow::OnGenerateWaypoints()` 中切换 |
| 修改面选取交互 | `src/ui/MainWindow.cpp` → `OnFacePicked()` / `OnSelectFaceMode()` |
| 修改轨迹规划逻辑 | `src/ui/TrajectoryPlanner.cpp` → `Plan()` / `InterpolateMoveL()` |
| 修改轨迹回放 | `src/ui/TrajectoryPlayer.cpp` + `MainWindow::OnPlaybackFrame()` |
| 修改轨迹编辑表格 | `src/ui/TrajectoryPanel.cpp` |
| 修改机器人/工件移动 | `src/ui/MovementPanel.cpp` + `MainWindow::OnMoveRobot/OnMoveWorkpiece()` |
| 新增 occ 模块功能 | `src/occ/` 下新建文件 + `src/occ/CMakeLists.txt` |
| 新增运动学功能 | `src/kinematics/` 下新建文件 + `src/kinematics/CMakeLists.txt` |

---

## MainWindow 成员速查

```cpp
// src/ui/MainWindow.h  (nl::ui::MainWindow)
OcctViewWidget*        viewer_            // 3D 视口
nl::core::RbRobot      current_robot_     // 已加载机器人数据
nl::utils::Q           joint_angles_      // 当前关节角（度），Q(6)
std::vector<RobotMesh> robot_meshes_      // 机器人各节 AIS 对象
QSlider*               joint_sliders_[6]  // Jog 滑块
QTreeWidget*           scene_tree_        // 场景树
std::vector<Handle(AIS_Trihedron)> joint_frames_  // 各关节坐标系（1-5 默认隐藏）
Handle(AIS_Trihedron)  base_frame_        // 机器人底座坐标系（默认显示）
Handle(AIS_Shape)      tool_ais_          // 工具 AIS 对象
Handle(AIS_Trihedron)  tool_tcp_frame_    // 工具 TCP 坐标系（默认显示）
gp_Trsf                tool_base_trsf_   // 工具安装偏移
gp_Trsf                tool_tcp_trsf_    // 工具 TCP 偏移
```

---

## 运动学约定

- **DH 约定**：Craig 1989（`Rz(θ) * Tz(d) * Tx(a) * Rx(α)`）
- **单位**：长度 mm，角度 degrees（内部转 radians）
- **关节顺序**：Joint1..Joint6，Joint6 = TCP
- **坐标系**：RobWork 世界系（Z 向上）

---

## 构建与测试

```bash
# CMake 路径（VS 内置）
CMAKE="D:/Program Files/Microsoft Visual Studio/2022/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"

# 配置（首次或 CMakeLists.txt 变更后）
"$CMAKE" -S . -B build -DVCPKG_MANIFEST_INSTALL=OFF

# 构建
"$CMAKE" --build build --config Release

# 测试
cd build && ctest --output-on-failure -C Release
```

---

## 外部依赖（vcpkg）

| 库 | 模块 | 用途 |
|---|---|---|
| Qt5 Core/Xml | GrindingCore | 数据类型、XML 解析 |
| Qt5 Gui/Widgets/OpenGL | GrindingUI | UI 框架 |
| Qt5 Test | Tests | 单元测试 |
| OpenCASCADE (OCCT) | GrindingOcc / GrindingKinematics | 3D 几何内核、STEP/STL、渲染 |
| Eigen3 | GrindingKinematics | 线性代数（DH 矩阵计算） |
