# GrindingApp — 项目架构地图

> 用途：减少 Claude 每次任务的探索 token。任务开始前先看此文件定位相关代码。

---

## 项目目标

Qt6 + OpenCASCADE (OCCT) 机器人磨削仿真应用。
- 加载工业机器人（RobWork `.rb.xml` 格式）、工具、工件
- 实时关节 Jog + 正运动学可视化（KDL）
- 场景树管理（Station Manager）

---

## 目录结构

```
GrindingApp/
├── src/                    # 所有 C++ 源码
│   ├── main.cpp            # 入口，QApplication + MainWindow
│   ├── MainWindow.h/cpp    # 主窗口：菜单、Jog面板、场景树、OCCT渲染
│   ├── OcctViewWidget.h/cpp# Qt Widget 封装 OCCT V3d_View
│   ├── RbXmlParser.h/cpp   # 解析 RobWork .rb.xml → RbRobot
│   ├── RobotDisplay.h/cpp  # 正运动学辅助（Craig DH）、RPY变换
│   ├── RobotKinematics.h/cpp # KDL FK 计算
│   ├── StlLoader.h/cpp     # 加载 .stl → TopoDS_Shape
│   └── StepImporter.h/cpp  # 加载 .step → TopoDS_Shape（工件用）
├── tests/
│   └── TestRobotKinematics.cpp  # Qt Test：KDL FK 验证
├── model/
│   ├── robot/IRB140/       # ABB IRB140 机器人模型
│   │   ├── IRB140.rb.xml   # RobWork 格式：DH参数 + 网格路径
│   │   └── Geometry/*.stl  # 7个 STL（BASE + LINK1..6）
│   └── tool/Burr/          # 打磨工具
│       ├── Burr.xml        # 工具坐标系定义（TCP0 偏移）
│       ├── Burr.tool       # 工具元数据（引用 Burr.xml）
│       └── Tool.stl        # 工具网格
├── resources/resources.qrc # Qt 资源（图标等）
├── translations/zh_CN.ts   # 中文翻译
├── CMakeLists.txt          # 构建配置
└── vcpkg.json              # 依赖声明
```

---

## 关键类与职责

| 类/模块 | 文件 | 职责 |
|---|---|---|
| `MainWindow` | `src/MainWindow.h/cpp` | 主窗口：UI 组装、场景树、Jog 面板、机器人/工具/工件加载 |
| `OcctViewWidget` | `src/OcctViewWidget.h/cpp` | 封装 OCCT 3D 视口，暴露 `Context()` / `View()` |
| `RbXmlParser` | `src/RbXmlParser.h/cpp` | 静态 `Parse(xml_path)` → `RbRobot`（含 DH 参数和 STL 路径） |
| `RobotDisplay` | `src/RobotDisplay.h/cpp` | `DhTrsf()`, `RpyPosTrsf()`, `ComputeFkHome()` |
| `RobotKinematics` | `src/RobotKinematics.h/cpp` | `ComputeFkKdl()` — 用 KDL 求各关节世界坐标系 |
| `StlLoader` | `src/StlLoader.h/cpp` | 静态 `Load(path)` → `TopoDS_Shape` |
| `StepImporter` | `src/StepImporter.h/cpp` | 静态 `Load(path)` → `TopoDS_Shape`（工件 STEP 导入） |

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
    double rpy[3];          // 欧拉角（RobWork 顺序：Rz*Ry*Rx）
    double pos[3];          // 平移 mm
    std::string mesh_file;  // .stl 绝对路径
};

struct RbRobot {
    std::string name;
    std::vector<RbJoint>    joints;      // 6个关节
    std::vector<RbDrawable> drawables;   // 7个网格（base+link1-6）
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
    │       └──► AIS_Trihedron → 坐标系渲染
    │
    └──► Jog Panel (QSlider)  → joint_angles_[] → UpdateRobotDisplay()
             └──► ComputeFkKdl() → 实时更新网格位置和坐标系
```

---

## 任务 → 文件速查表

| 任务 | 主要文件 |
|---|---|
| 修改 FK 计算逻辑 | `src/RobotKinematics.cpp` |
| 修改 Jog 面板 UI | `src/MainWindow.cpp` → `SetupJogPanel()` |
| 修改关节坐标系渲染 | `src/MainWindow.cpp` → `UpdateCoordinateFrames()` |
| 修改场景树 | `src/MainWindow.cpp` → `SetupSceneTree()`, `AddRobot()`, `AddTool()` |
| 添加 IK | 新建 `src/RobotKinematics.cpp` → `ComputeIkKdl()` |
| 加载新格式机器人 | `src/RbXmlParser.h/cpp` |
| 修改工具加载 | `src/MainWindow.cpp` → `OnLoadTool()` |
| 修改工件加载 | `src/MainWindow.cpp` → `OnImportWorkpiece()` + `src/StepImporter.h/cpp` |
| 修改 3D 视口交互 | `src/OcctViewWidget.cpp` |
| 添加 STL 加载逻辑 | `src/StlLoader.h/cpp` |
| 修改 OCCT 显示风格 | `src/MainWindow.cpp` → `OnViewWireframe()` / `OnViewShaded()` |
| 添加新测试 | `tests/TestRobotKinematics.cpp` |

---

## 构建与测试

```bash
# 构建
cmake --build build --config Release

# 测试
cd build && ctest --output-on-failure

# 构建目录
build/bin/GrindingApp.exe    # 主程序
build/bin/TestRobotKinematics.exe  # 测试
```

---

## 外部依赖（vcpkg）

| 库 | 用途 |
|---|---|
| Qt5 (Core/Gui/Widgets/OpenGL/Xml/Test) | UI 框架、XML 解析、测试 |
| OpenCASCADE (OCCT) | 3D 几何内核、STEP/STL 加载、渲染 |
| orocos-kdl | 机器人正/逆运动学（KDL Chain） |

---

## MainWindow 成员速查

```cpp
// src/MainWindow.h
OcctViewWidget*        viewer_            // 3D 视口
RbRobot                current_robot_     // 已加载机器人数据
double                 joint_angles_[6]   // 当前关节角（度）
std::vector<RobotMesh> robot_meshes_      // 机器人各节 AIS 对象
QSlider*               joint_sliders_[6]  // Jog 滑块
QTreeWidget*           scene_tree_        // 场景树
std::vector<Handle(AIS_Trihedron)> joint_frames_  // 各关节坐标系
Handle(AIS_Trihedron)  base_frame_        // 机器人底座坐标系
Handle(AIS_Shape)      tool_ais_          // 工具 AIS 对象
Handle(AIS_Trihedron)  tool_tcp_frame_    // 工具 TCP 坐标系
gp_Trsf                tool_base_trsf_   // 工具安装偏移
gp_Trsf                tool_tcp_trsf_    // 工具 TCP 偏移
```

---

## 运动学约定

- **DH 约定**：Craig 1989（先旋转后平移：`Rz(θ) * Tz(d) * Tx(a) * Rx(α)`）
- **单位**：长度 mm，角度 degrees（KDL 内部转 radians）
- **关节顺序**：Joint1..Joint6，Joint6 = TCP
- **坐标系**：RobWork 世界系（Z 向上）
