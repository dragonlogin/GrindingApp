# GrindingApp — 项目架构地图

> 用途：减少 Claude 每次任务的探索 token。任务开始前先看此文件定位相关代码。

---

## 项目目标

Qt5 + OpenCASCADE (OCCT) 机器人磨削仿真应用。
- 加载工业机器人（RobWork `.rb.xml` 格式）、工具、工件
- 实时关节 Jog + 正运动学可视化（KDL）
- 场景树管理（Station Manager）

---

## 构建产物

| 目标 | 类型 | 路径 |
|---|---|---|
| `GrindingAppLib` | SHARED DLL | `build/bin/Release/GrindingAppLib.dll` |
| `GrindingApp` | Executable | `build/bin/Release/GrindingApp.exe` |
| `TestRobotKinematics` | Test Executable | `build/tests/Release/TestRobotKinematics.exe` |

- `GrindingApp` 仅包含 `execute/main.cpp`，链接 `GrindingAppLib`
- 测试直接链接 `GrindingAppLib`（继承所有 PUBLIC 依赖，无需重复声明）

---

## 目录结构

```
GrindingApp/
├── execute/
│   └── main.cpp              # 可执行入口：QApplication + MainWindow
├── src/                      # GrindingAppLib 库源码
│   ├── GrindingAppExport.h   # DLL 导出宏（GRINDINGAPP_EXPORT）
│   ├── RbXmlParser.h/cpp     # 共享数据类型：RbRobot / RbJoint / RbDrawable
│   ├── ui/                   # Qt UI 层
│   │   ├── MainWindow.h/cpp  # 主窗口：菜单、Jog面板、场景树、OCCT渲染
│   │   └── OcctViewWidget.h/cpp  # Qt Widget 封装 OCCT V3d_View
│   ├── occ/                  # OpenCASCADE 操作层
│   │   ├── StlLoader.h/cpp   # 加载 .stl → TopoDS_Shape
│   │   ├── StepImporter.h/cpp# 加载 .step → TopoDS_Shape（工件用）
│   │   └── RobotDisplay.h/cpp# DhTrsf / RpyPosTrsf / ComputeFkHome
│   └── kdl/                  # KDL 运动学层
│       └── RobotKinematics.h/cpp  # ComputeFkKdl（KDL FK）
├── tests/
│   ├── CMakeLists.txt        # 独立测试构建：链接 GrindingAppLib
│   └── TestRobotKinematics.cpp  # Qt Test：KDL FK vs 手工 DH 对比
├── model/
│   ├── robot/IRB140/         # ABB IRB140 机器人模型
│   │   ├── IRB140.rb.xml     # RobWork 格式：DH参数 + 网格路径
│   │   └── Geometry/*.stl    # 7个 STL（BASE + LINK1..6）
│   └── tool/Burr/            # 打磨工具
│       ├── Burr.xml          # 工具坐标系定义（TCP0 偏移）
│       ├── Burr.tool         # 工具元数据（引用 Burr.xml）
│       └── Tool.stl          # 工具网格
├── resources/resources.qrc   # Qt 资源（编译进 DLL）
├── translations/zh_CN.ts     # 中文翻译源文件
├── CMakeLists.txt            # 根构建配置
└── vcpkg.json                # vcpkg 依赖声明
```

---

## 模块职责

### `src/` 根 — 共享数据类型

| 文件 | 职责 |
|---|---|
| `GrindingAppExport.h` | `GRINDINGAPP_EXPORT` 宏：构建时 `dllexport`，使用时 `dllimport` |
| `RbXmlParser.h/cpp` | 静态 `Parse(xml_path)` → `RbRobot`（含 DH 参数和 STL 路径） |

### `src/ui/` — Qt UI 层

| 文件 | 职责 |
|---|---|
| `MainWindow.h/cpp` | 主窗口：UI 组装、场景树、Jog 面板、机器人/工具/工件加载 |
| `OcctViewWidget.h/cpp` | 封装 OCCT 3D 视口，暴露 `Context()` / `View()` / `Viewer()` |

### `src/occ/` — OpenCASCADE 操作层

| 文件 | 职责 |
|---|---|
| `StlLoader.h/cpp` | 静态 `Load(path)` → `TopoDS_Shape`（RWStl 读取） |
| `StepImporter.h/cpp` | 静态 `Load(path, face_count*)` → `TopoDS_Shape`（工件 STEP 导入） |
| `RobotDisplay.h/cpp` | `DhTrsf()`、`RpyPosTrsf()`、`ComputeFkHome()` |

### `src/kdl/` — KDL 运动学层

| 文件 | 职责 |
|---|---|
| `RobotKinematics.h/cpp` | `ComputeFkKdl(robot, angles[6])` → `gp_Trsf × 6`（各关节世界坐标系） |

---

## 核心数据结构

```cpp
// src/RbXmlParser.h
struct RbJoint {
    std::string name;
    double alpha_deg, a, d, offset_deg;  // Craig DH 参数（a/d 单位 mm）
};

struct RbDrawable {
    std::string name;
    std::string ref_joint;  // "Robot_Base" | "Joint1".."Joint6"
    double rpy[3];          // 欧拉角（RobWork 顺序：Rz*Ry*Rx），单位 deg
    double pos[3];          // 平移，单位 mm
    std::string mesh_file;  // .stl 绝对路径
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
    ├──► ComputeFkKdl()       → gp_Trsf × 6（关节世界坐标）
    │       └──► AIS_Trihedron → 坐标系渲染（base/tcp 默认显示）
    │
    └──► Jog Panel (QSlider)  → joint_angles_[] → UpdateRobotDisplay()
             └──► ComputeFkKdl() → 实时更新网格位置和坐标系
```

---

## 任务 → 文件速查表

| 任务 | 主要文件 |
|---|---|
| 修改 FK 计算逻辑 | `src/kdl/RobotKinematics.cpp` |
| 添加 IK | `src/kdl/RobotKinematics.h/cpp` → 新增 `ComputeIkKdl()` |
| 修改 Jog 面板 UI | `src/ui/MainWindow.cpp` → `SetupJogPanel()` |
| 修改关节坐标系渲染 | `src/ui/MainWindow.cpp` → `UpdateCoordinateFrames()` |
| 修改场景树 | `src/ui/MainWindow.cpp` → `SetupSceneTree()`, `AddRobot()`, `AddTool()` |
| 加载新格式机器人 | `src/RbXmlParser.h/cpp` |
| 修改工具加载 | `src/ui/MainWindow.cpp` → `OnLoadTool()` |
| 修改工件加载 | `src/ui/MainWindow.cpp` → `OnImportWorkpiece()` + `src/occ/StepImporter.h/cpp` |
| 修改 3D 视口交互 | `src/ui/OcctViewWidget.cpp` |
| 修改 STL 加载逻辑 | `src/occ/StlLoader.h/cpp` |
| 修改 OCCT 显示风格 | `src/ui/MainWindow.cpp` → `OnViewWireframe()` / `OnViewShaded()` |
| 添加新测试 | `tests/TestRobotKinematics.cpp` + `tests/CMakeLists.txt` |

---

## MainWindow 成员速查

```cpp
// src/ui/MainWindow.h
OcctViewWidget*        viewer_            // 3D 视口
RbRobot                current_robot_     // 已加载机器人数据
double                 joint_angles_[6]   // 当前关节角（度）
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
- **单位**：长度 mm，角度 degrees（KDL 内部转 radians）
- **关节顺序**：Joint1..Joint6，Joint6 = TCP
- **坐标系**：RobWork 世界系（Z 向上）

---

## 构建与测试

```bash
# CMake 路径（VS 内置）
CMAKE="D:/Program Files/Microsoft Visual Studio/2022/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"

# 构建
"$CMAKE" --build build --config Release

# 测试
cd build && ctest --output-on-failure -C Release
```

---

## 外部依赖（vcpkg）

| 库 | 用途 |
|---|---|
| Qt5 (Core/Gui/Widgets/OpenGL/Xml/Test) | UI 框架、XML 解析、测试 |
| OpenCASCADE (OCCT) | 3D 几何内核、STEP/STL 加载、渲染 |
| orocos-kdl | 机器人正/逆运动学（KDL Chain） |
| Eigen3 | 线性代数（KDL 依赖） |
