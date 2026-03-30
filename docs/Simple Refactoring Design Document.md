# GrindingApp 重构设计文档：Qt 解耦的后端内核架构

## Summary

本次重构的目标是把当前“Qt UI 承担应用流程编排”的结构，重构为“后端内核 + 前端适配器”结构，并且从一开始就为以下能力留出架构空间：

- 项目保存 / 打开
- 多场景管理
- 后续接入 OMPL 规划
- 后续接入碰撞检测
- 后续替换或新增前端，而不影响核心逻辑

本设计采用“单进程内嵌核心优先”的路线，不引入 COM。  
第一阶段保留 Qt 作为桌面前端，但 Qt 只负责界面和交互，不再直接承载主流程。  
后续如需做独立本地服务，应在本设计基础上新增 RPC/IPC 适配层，而不是再拆核心。

---

## Design Goals

### 必须满足
- 后端不依赖 Qt Widgets / QObject / signal-slot 才能工作
- 导入、路径生成、规划、保存、打开能够在不启动 Qt 的情况下运行
- 一个项目可包含多个场景
- Geometry、Planning、Persistence 能独立演进
- 后续替换轨迹规划器实现时，应用层调用方式不变

### 明确非目标
- 第一阶段不强制实现独立进程服务
- 第一阶段不实现完整 OMPL 和完整碰撞检测
- 第一阶段不追求彻底消灭 OCCT，而是把 OCCT 放到正确层次
- 第一阶段不保存 Qt 布局状态，保存的是项目和场景状态

---

## Layered Architecture

### 1. `foundation`
最底层基础设施，纯 C++。

职责：
- 基础值类型
- 坐标/单位辅助
- 错误类型
- 资源标识与句柄
- 通用结果返回模型

包含内容：
- `Vector3d`
- `Q`
- `Result<T>`
- `ErrorCode`
- `SceneId` / `ProjectId` / `ObjectId`
- 单位转换辅助

约束：
- 不依赖 Qt
- 不依赖 OCCT
- 不依赖 OMPL
- 不包含业务语义

---

### 2. `domain`
纯业务模型与业务规则层。

职责：
- 表达系统状态
- 约束状态合法性
- 定义项目、场景、对象之间关系
- 为保存/打开提供稳定内部模型

核心对象：
- `Project`
- `Scene`
- `SceneObject`
- `Robot`
- `Tool`
- `Workpiece`
- `WaypointSet`
- `Trajectory`
- `PlanningProfile`
- `CollisionProfile`

建议模型边界：

#### `Project`
包含：
- `ProjectId`
- `std::string name`
- `std::vector<Scene>`
- 全局设置
- 资源引用表

职责：
- 管理多个 `Scene`
- 提供按 `SceneId` 查找和增删场景
- 作为保存/打开的根对象

#### `Scene`
包含：
- `SceneId`
- 名称
- 当前机器人
- 当前工具
- 当前工件
- 夹具/附加对象列表
- 当前路径点集
- 当前轨迹
- 当前选中的几何引用
- 机器人 base pose
- 工件 pose
- 规划与碰撞配置

职责：
- 表达一个完整工作站场景
- 作为规划、碰撞、回放的边界
- 保证内部状态一致性

#### `Robot`
包含：
- 机器人名称
- 关节列表
- 关节限制
- 视觉 mesh 资源引用
- 碰撞 mesh 资源引用
- 运动学元数据
- 来源元信息

职责：
- 表达系统内部机器人模型
- 不关心它来自 URDF 还是项目恢复

#### `Tool`
包含：
- 名称
- 几何资源引用
- base 安装位姿
- tcp 位姿
- 质量/半径等后续工艺参数

#### `Workpiece`
包含：
- 几何资源引用
- 当前场景变换
- 选中面引用
- 工艺属性

#### `WaypointSet`
包含：
- waypoint 列表
- 来源面引用
- 生成配置
- 归属 `SceneId`

#### `Trajectory`
包含：
- 轨迹点列表
- 规划配置快照
- 规划状态
- 错误标记
- 来源 waypoint 集 ID

约束：
- `domain` 不能依赖解析器
- `domain` 不能依赖 Qt 类型
- `domain` 不直接依赖具体文件格式

---

### 3. `model_import`
模型导入导出与项目序列化层。

职责：
- 解析外部文件格式
- 将外部表示转换为内部定义
- 将内部项目保存为项目文件
- 从项目文件恢复项目

核心接口：

#### `IRobotImporter`
```cpp
Result<RobotDefinition> ImportRobot(const std::string& path);
```

#### `IToolImporter`
```cpp
Result<ToolDefinition> ImportTool(const std::string& path);
```

#### `IProjectSerializer`
```cpp
Result<ProjectStorageModel> LoadProject(const std::string& path);
Result<void> SaveProject(const std::string& path, const ProjectStorageModel& model);
```

中间模型：
- `RobotDefinition`
- `ToolDefinition`
- `ProjectStorageModel`

说明：
- `RobotDefinition` 是“从文件解析出来的中间结构”
- `Domain` 再基于 definition 构造 `Robot`
- 当前 [RbXmlParser.cpp](E:/Code/GrindingApp/src/core/RbXmlParser.cpp) 不应继续位于核心层，应迁入这里
- 虽然名字里现在叫 `RbXmlParser`，但它实际已经承担 URDF 导入，应改成格式无关的 importer 语义

约束：
- 可依赖 Qt XML 或其他解析库，但依赖不能向上泄漏
- 不持有 UI 状态
- 不负责场景业务编排

---

### 4. `geometry`
独立几何子系统，主要封装 OCCT。

职责：
- STEP / mesh / shape 导入
- 拓扑查询
- 稳定的面/边/shape 引用
- 几何变换
- 工件相关路径点生成
- 后续为碰撞与规划场景提供几何输入

核心接口：

#### `IGeometryImporter`
```cpp
Result<GeometryAsset> ImportStep(const std::string& path);
Result<GeometryAsset> ImportMesh(const std::string& path);
```

#### `IShapeQueryService`
```cpp
Result<std::vector<FaceRef>> ListFaces(const GeometryHandle& handle);
Result<FaceProperties> GetFaceProperties(const GeometryHandle& handle, const FaceRef& face);
```

#### `IWaypointGenerator`
```cpp
Result<WaypointSet> Generate(const WorkpieceGeometryContext& context,
                             const FaceRef& face,
                             const WaypointGenerationConfig& config);
```

#### `IGeometryTransformService`
```cpp
Pose Compose(const Pose& a, const Pose& b);
Pose Inverse(const Pose& pose);
```

核心数据：
- `GeometryHandle`
- `FaceRef`
- `GeometryAsset`
- `FaceProperties`
- `WaypointGenerationConfig`

设计要求：
- `FaceRef` 必须是稳定引用，不能让 UI 长期持有 `TopoDS_Face`
- `GeometryHandle` 由 geometry 子系统统一管理生命周期
- `Waypoint` 和 `Trajectory` 不再放在 `occ` 命名空间下，而作为 domain 值对象

当前映射：
- [StepImporter.cpp](E:/Code/GrindingApp/src/occ/StepImporter.cpp) -> `geometry`
- [MeshLoader.cpp](E:/Code/GrindingApp/src/occ/MeshLoader.cpp) -> `geometry`
- [WaypointGenerator.cpp](E:/Code/GrindingApp/src/occ/WaypointGenerator.cpp) -> `geometry`
- [WaypointGridAlgo.cpp](E:/Code/GrindingApp/src/occ/WaypointGridAlgo.cpp) -> `geometry`
- [WaypointPlanarAlgo.cpp](E:/Code/GrindingApp/src/occ/WaypointPlanarAlgo.cpp) -> `geometry`
- [SurfaceWaypointGen.cpp](E:/Code/GrindingApp/src/occ/SurfaceWaypointGen.cpp) -> `geometry`
- [Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h) -> `domain`
- [Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h) -> `domain`

---

### 5. `planning`
运动学、轨迹规划、碰撞和验证层。

职责：
- FK / IK
- 轨迹插值
- 后续 OMPL 规划
- 路径合法性验证
- 碰撞检测
- 规划场景构建

核心接口：

#### `IKinematicsService`
```cpp
Result<std::vector<Pose>> ComputeFk(const Robot& robot, const JointState& joints);
Result<JointState> ComputeIk(const Robot& robot, const Pose& target, const JointState& seed);
Result<std::vector<JointState>> ComputeIkAll(const Robot& robot, const Pose& target, const JointState& seed);
```

#### `ITrajectoryPlanner`
```cpp
Result<Trajectory> Plan(const PlanningSceneSnapshot& scene,
                        const WaypointSet& waypoints,
                        const PlanningRequest& request);
```

#### `ICollisionChecker`
```cpp
Result<CollisionReport> CheckScene(const PlanningSceneSnapshot& scene);
Result<CollisionReport> CheckTrajectory(const PlanningSceneSnapshot& scene,
                                        const Trajectory& trajectory);
```

#### `IPlanningSceneBuilder`
```cpp
Result<PlanningSceneSnapshot> Build(const Scene& scene);
```

规划子层建议：
- `planning/kinematics`
- `planning/trajectory`
- `planning/collision`
- `planning/ompl`（后续）

当前映射：
- [IKinematicsSolver.h](E:/Code/GrindingApp/src/kinematics/IKinematicsSolver.h) -> `planning/kinematics`
- [KdlSolver.cpp](E:/Code/GrindingApp/src/kinematics/KdlSolver.cpp) -> `planning/kinematics`
- [KdlChainBuilder.cpp](E:/Code/GrindingApp/src/kinematics/KdlChainBuilder.cpp) -> `planning/kinematics`
- [RobotKinematics.cpp](E:/Code/GrindingApp/src/kinematics/RobotKinematics.cpp) -> `planning/kinematics facade`
- [TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp) -> `planning/trajectory`

要求：
- `TrajectoryPlanner` 不能继续放在 `ui`
- `planning` 不依赖 Qt
- `planning` 可依赖 geometry 提供的场景几何快照，但不依赖 UI 组件

---

### 6. `application`
应用编排层，是新的系统主入口。

职责：
- 编排 domain、model_import、geometry、planning
- 执行用户用例
- 管理项目与场景状态
- 生成前端需要的快照数据
- 统一错误返回和操作结果

主服务建议：

#### `IProjectAppService`
```cpp
Result<ProjectId> CreateProject(const CreateProjectRequest& req);
Result<ProjectSnapshot> OpenProject(const OpenProjectRequest& req);
Result<void> SaveProject(const SaveProjectRequest& req);
Result<SceneId> CreateScene(const CreateSceneRequest& req);
Result<ProjectSnapshot> GetProjectSnapshot(ProjectId id) const;
```

#### `ISceneAppService`
```cpp
Result<SceneSnapshot> LoadRobot(const LoadRobotRequest& req);
Result<SceneSnapshot> LoadTool(const LoadToolRequest& req);
Result<SceneSnapshot> ImportWorkpiece(const ImportWorkpieceRequest& req);
Result<SceneSnapshot> SelectFace(const SelectFaceRequest& req);
Result<SceneSnapshot> GenerateWaypoints(const GenerateWaypointsRequest& req);
Result<SceneSnapshot> PlanTrajectory(const PlanTrajectoryRequest& req);
Result<SceneSnapshot> SetRobotBasePose(const SetRobotBasePoseRequest& req);
Result<SceneSnapshot> SetWorkpiecePose(const SetWorkpiecePoseRequest& req);
Result<TcpPoseSnapshot> SetTcpPose(const SetTcpPoseRequest& req);
Result<SceneSnapshot> SetJointState(const SetJointStateRequest& req);
Result<SceneSnapshot> ClearTrajectory(const ClearTrajectoryRequest& req);
```

内部组成建议：
- `ProjectRepository`
- `SceneSessionManager`
- `SceneAssembler`
- `SnapshotBuilder`

关键原则：
- UI 不直接调 `kinematics::ComputeIk`
- UI 不直接调 `occ::StepImporter`
- UI 不直接持有业务状态真源
- 所有主流程都进 `application`

---

### 7. `adapters/qt_desktop`
Qt 桌面适配层。

职责：
- 菜单和 Dock
- 文件对话框
- 场景树
- 3D 视图
- 用户输入转发
- 响应应用层结果并更新显示

保留文件：
- [MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp)
- [MainWindow.h](E:/Code/GrindingApp/src/ui/MainWindow.h)
- [JogPanel.cpp](E:/Code/GrindingApp/src/ui/JogPanel.cpp)
- [TrajectoryPanel.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPanel.cpp)
- [TrajectoryPlayer.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlayer.cpp)
- [MovementPanel.cpp](E:/Code/GrindingApp/src/ui/MovementPanel.cpp)
- [OcctViewWidget.cpp](E:/Code/GrindingApp/src/ui/OcctViewWidget.cpp)
- [Commands.cpp](E:/Code/GrindingApp/src/ui/Commands.cpp)

必须拆出的部分：
- `MainWindow` 中的业务编排逻辑
- `RobotController` 中的模型加载和状态真源
- `TrajectoryPlanner` 中的规划逻辑

Qt 适配层分成两个角色：

#### `QtScenePresenter`
- 把 `SceneSnapshot` 映射到 tree / labels / panel
- 不持有真实 domain 状态

#### `QtOccViewportAdapter`
- 管理 AIS 对象生命周期
- 把 geometry / scene snapshot 转成可视化内容
- 可持有 `AIS_Shape`，但这些不是业务数据真源

---

## Dependency Direction

必须严格遵守以下依赖方向：

- `foundation` 无上游依赖
- `domain` -> `foundation`
- `model_import` -> `foundation`, `domain`
- `geometry` -> `foundation`, `domain`
- `planning` -> `foundation`, `domain`, `geometry` 的抽象或快照
- `application` -> `foundation`, `domain`, `model_import` 抽象, `geometry` 抽象, `planning` 抽象
- `adapters/qt_desktop` -> `application`

禁止：
- `domain` 依赖 Qt / OCCT / OMPL
- `application` 依赖 QWidget/AIS 细节
- `Qt` 直接依赖底层算法函数而绕过 `application`
- `Trajectory` / `Waypoint` 长期挂在 `occ` 命名空间里

---

## Module Communication

### 1. 主通信方式：同步接口调用
模块之间主要使用：
- 纯 C++ 接口
- DTO / 值对象
- `Result<T>` 返回

理由：
- 当前是单进程内嵌架构
- 调试最简单
- 后续服务化时容易外包一层 RPC

---

### 2. 状态传递方式：Snapshot
前端不直接拿 domain 可变对象，而是消费快照：

- `ProjectSnapshot`
- `SceneSnapshot`
- `TcpPoseSnapshot`
- `TrajectorySnapshot`

快照内容：
- 可直接显示的数据
- 稳定 ID
- 轻量结构化状态
- 不包含底层 OCCT 句柄和 Qt 对象

理由：
- 前后端边界稳定
- 有利于保存/恢复/撤销/服务化
- 避免前端直接篡改核心状态

---

### 3. 几何引用方式：Handle + Ref
geometry 模块对外提供：
- `GeometryHandle`
- `FaceRef`

Qt 只传回这些稳定标识，不传 `TopoDS_Face` 长期作为业务对象。  
`TopoDS_Face` 只应停留在 geometry / viewport adapter 内部。

---

### 4. UI 更新方式：应用结果驱动
不再以 `RobotController::DisplayUpdated` 这种 UI 控制器为真源。  
改为：

1. UI 发请求给 `application`
2. `application` 更新项目/场景
3. 返回新的 `SceneSnapshot`
4. presenter 和 viewport adapter 分别刷新

---

### 5. 后续跨进程扩展方式
若未来做本地服务，新增：
- `adapters/local_rpc_server`
- `adapters/local_rpc_client`

保持不变：
- `domain`
- `application`
- `geometry`
- `planning`

变化：
- 同步接口被 RPC facade 包装
- Snapshot / Request / Response 变成序列化 DTO

因此当前不需要 COM。

---

## Data Model and Key DTOs

### Domain value objects

#### `Pose`
建议新增统一位姿类型，代替到处裸用 `gp_Trsf`
```cpp
struct Pose {
    double matrix[16];
};
```
或内部允许适配到 OCCT，但对外接口不直接暴露 `gp_Trsf`。

理由：
- 降低 Domain / Application 对 OCCT 的直接耦合
- 服务化和序列化更容易

#### `JointState`
用当前 `Q` 演进而来
```cpp
struct JointState {
    std::vector<double> values_deg;
};
```

#### `Waypoint`
从 `occ` 命名空间迁出
```cpp
struct Waypoint {
    Pose pose;
    double speed_ratio;
};
```

#### `TrajectoryPoint`
```cpp
struct TrajectoryPoint {
    Pose tcp_pose;
    JointState joints;
    MoveType move_type;
    PointStatus status;
    int waypoint_index;
};
```

---

### Application DTOs

#### `LoadRobotRequest`
```cpp
struct LoadRobotRequest {
    SceneId scene_id;
    std::string file_path;
};
```

#### `ImportWorkpieceRequest`
```cpp
struct ImportWorkpieceRequest {
    SceneId scene_id;
    std::string file_path;
};
```

#### `GenerateWaypointsRequest`
```cpp
struct GenerateWaypointsRequest {
    SceneId scene_id;
    FaceRef face_ref;
    WaypointGenerationConfig config;
};
```

#### `PlanTrajectoryRequest`
```cpp
struct PlanTrajectoryRequest {
    SceneId scene_id;
    PlanningRequest planning;
};
```

#### `SceneSnapshot`
至少包含：
- scene id / name
- robot summary
- tool summary
- workpiece summary
- current joints
- current tcp pose
- selected face ref
- waypoint count
- trajectory summary
- object transforms
- planning status

---

## Save / Open Design

### 保存对象
保存的是 `Project`，不是 `MainWindow` 状态。

项目文件至少应覆盖：
- 项目元数据
- scene 列表
- 每个 scene 中的机器人、工具、工件引用
- base pose / workpiece pose
- 当前 joint state
- 选中面引用
- waypoint 数据
- trajectory 数据
- 当前 planner config
- 预留 collision config / OMPL config

### 路径策略
建议保存：
- 原始资源路径
- 可选资源 ID
- 统一相对路径基准

避免：
- 只保存 UI 中的临时内存状态
- 直接保存 Qt widget 结构

### 打开流程
1. serializer 读取项目文件
2. application 构造 `Project`
3. model import / geometry 恢复资源
4. scene 重新建立运行时几何句柄
5. 返回 `ProjectSnapshot`

---

## Multi-Scene Design

`Project` 下允许多个 `Scene`。  
每个 `Scene` 独立持有：
- robot
- tool
- workpiece
- waypoints
- trajectory
- 规划配置
- 碰撞配置

应用层必须支持：
- 创建场景
- 切换场景
- 删除场景
- 复制场景
- 查询当前活动场景

Qt 第一阶段即便只显示一个活动场景，底层也必须按多场景设计。

---

## OMPL and Collision Integration Design

### OMPL 接入位置
放在 `planning/ompl`，实现 `ITrajectoryPlanner`。

Application 不关心具体 planner 是：
- 简单插值
- OMPL
- 其他规划器

只关心：
- 输入：`PlanningSceneSnapshot + WaypointSet + PlanningRequest`
- 输出：`Trajectory`

### 碰撞检测位置
放在 `planning/collision`，实现 `ICollisionChecker`。

碰撞输入来源：
- `Scene`
- geometry 生成的碰撞几何
- robot/tool/workpiece transforms

### PlanningSceneSnapshot
规划器和碰撞检测都通过统一快照消费场景：
- robot state
- tool pose
- workpiece pose
- geometry handles / collision geometry refs
- constraints
- planner settings

这样 OMPL 和 collision 不需要直接依赖 Qt 或项目文件格式。

---

## Concrete Mapping From Current Files

### 迁入 `foundation`
- [Vector3d.h](E:/Code/GrindingApp/src/utils/Vector3d.h)
- [Q.h](E:/Code/GrindingApp/src/utils/Q.h)
- [Q.cpp](E:/Code/GrindingApp/src/utils/Q.cpp)

### 拆分 `core`
迁入 `model_import`
- [RbXmlParser.h](E:/Code/GrindingApp/src/core/RbXmlParser.h)
- [RbXmlParser.cpp](E:/Code/GrindingApp/src/core/RbXmlParser.cpp)

迁入 `domain`
- `RbRobot` / `RbJoint` / `RbDrawable` 对应的稳定业务结构，但要去除 `RbXml` 语义并重命名

### 迁入 `geometry`
- [MeshLoader.h](E:/Code/GrindingApp/src/occ/MeshLoader.h)
- [MeshLoader.cpp](E:/Code/GrindingApp/src/occ/MeshLoader.cpp)
- [StepImporter.h](E:/Code/GrindingApp/src/occ/StepImporter.h)
- [StepImporter.cpp](E:/Code/GrindingApp/src/occ/StepImporter.cpp)
- [WaypointGenerator.h](E:/Code/GrindingApp/src/occ/WaypointGenerator.h)
- [WaypointGenerator.cpp](E:/Code/GrindingApp/src/occ/WaypointGenerator.cpp)
- [WaypointGridAlgo.h](E:/Code/GrindingApp/src/occ/WaypointGridAlgo.h)
- [WaypointGridAlgo.cpp](E:/Code/GrindingApp/src/occ/WaypointGridAlgo.cpp)
- [WaypointPlanarAlgo.h](E:/Code/GrindingApp/src/occ/WaypointPlanarAlgo.h)
- [WaypointPlanarAlgo.cpp](E:/Code/GrindingApp/src/occ/WaypointPlanarAlgo.cpp)
- [SurfaceWaypointGen.h](E:/Code/GrindingApp/src/occ/SurfaceWaypointGen.h)
- [SurfaceWaypointGen.cpp](E:/Code/GrindingApp/src/occ/SurfaceWaypointGen.cpp)

### 迁入 `domain`
- [Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h)
- [Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h)

### 迁入 `planning`
- [IKinematicsSolver.h](E:/Code/GrindingApp/src/kinematics/IKinematicsSolver.h)
- [EigenSolver.h](E:/Code/GrindingApp/src/kinematics/EigenSolver.h)
- [EigenSolver.cpp](E:/Code/GrindingApp/src/kinematics/EigenSolver.cpp)
- [KdlChainBuilder.h](E:/Code/GrindingApp/src/kinematics/KdlChainBuilder.h)
- [KdlChainBuilder.cpp](E:/Code/GrindingApp/src/kinematics/KdlChainBuilder.cpp)
- [KdlSolver.h](E:/Code/GrindingApp/src/kinematics/KdlSolver.h)
- [KdlSolver.cpp](E:/Code/GrindingApp/src/kinematics/KdlSolver.cpp)
- [RobotKinematics.h](E:/Code/GrindingApp/src/kinematics/RobotKinematics.h)
- [RobotKinematics.cpp](E:/Code/GrindingApp/src/kinematics/RobotKinematics.cpp)
- [TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h)
- [TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp)

### 保留在 `adapters/qt_desktop`
- [MainWindow.h](E:/Code/GrindingApp/src/ui/MainWindow.h)
- [MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp)
- [JogPanel.h](E:/Code/GrindingApp/src/ui/JogPanel.h)
- [JogPanel.cpp](E:/Code/GrindingApp/src/ui/JogPanel.cpp)
- [TrajectoryPanel.h](E:/Code/GrindingApp/src/ui/TrajectoryPanel.h)
- [TrajectoryPanel.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPanel.cpp)
- [TrajectoryPlayer.h](E:/Code/GrindingApp/src/ui/TrajectoryPlayer.h)
- [TrajectoryPlayer.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlayer.cpp)
- [MovementPanel.h](E:/Code/GrindingApp/src/ui/MovementPanel.h)
- [MovementPanel.cpp](E:/Code/GrindingApp/src/ui/MovementPanel.cpp)
- [OcctViewWidget.h](E:/Code/GrindingApp/src/ui/OcctViewWidget.h)
- [OcctViewWidget.cpp](E:/Code/GrindingApp/src/ui/OcctViewWidget.cpp)
- [Commands.h](E:/Code/GrindingApp/src/ui/Commands.h)
- [Commands.cpp](E:/Code/GrindingApp/src/ui/Commands.cpp)

### 需要拆分的混合文件
- [RobotController.h](E:/Code/GrindingApp/src/ui/RobotController.h)
- [RobotController.cpp](E:/Code/GrindingApp/src/ui/RobotController.cpp)

拆分目标：
- `application/scene_app_service`
- `adapters/qt_desktop/occ_scene_renderer`

---

## Migration Plan

### Phase 1: 建立新 target 和边界
- 新增 CMake targets：
  - `GrindingFoundation`
  - `GrindingDomain`
  - `GrindingModelImport`
  - `GrindingGeometry`
  - `GrindingPlanning`
  - `GrindingApplication`
  - `GrindingQtDesktop`
- 暂时允许旧 UI 和新模块并存
- 不立即改全部调用点

验收：
- 新 target 可独立编译
- 依赖方向符合设计

---

### Phase 2: 抽离 domain 和 foundation
- 把 `Q`、`Vector3d` 移到 foundation
- 把 `Waypoint`、`Trajectory`、机器人内部模型迁到 domain
- 用类型别名或兼容头降低迁移成本

验收：
- 现有 kinematics/trajectory/geometry 可用新 domain 类型编译

---

### Phase 3: 抽离 model_import
- 将 `RbXmlParser` 改造成 `IRobotImporter` 实现
- 引入 `RobotDefinition`
- `LoadRobot` 不再由 UI 直接调用 parser

验收：
- 不启动 Qt，可调用 importer 获取机器人定义

---

### Phase 4: 抽离 planning
- 将 `TrajectoryPlanner` 从 UI 移出
- `RobotKinematics` 提供清晰的 `IKinematicsService`
- 让 trajectory planning 不依赖 QWidget

验收：
- 可写纯后端测试覆盖“waypoints -> trajectory”

---

### Phase 5: 引入 application
- 新建 `ProjectAppService` 和 `SceneAppService`
- 实现：
  - `LoadRobot`
  - `LoadTool`
  - `ImportWorkpiece`
  - `GenerateWaypoints`
  - `PlanTrajectory`
  - `SetJointState`
  - `SetTcpPose`
- UI 改为只调用 app service

验收：
- 主流程可在不依赖 `MainWindow` 逻辑的情况下跑通

---

### Phase 6: 重写 Qt 适配方式
- `MainWindow` 只保留 UI 事件和渲染协调
- `RobotController` 拆成：
  - `SceneAppService` 调用方
  - `OccSceneRenderer`
- Scene tree 和 panels 改为消费 `SceneSnapshot`

验收：
- `MainWindow` 不再直接调用 `ComputeIk` / `StepImporter` / `WaypointGenerator`

---

### Phase 7: 保存/打开与多场景
- 引入 `Project` 和 `Scene`
- 增加 serializer
- 增加多 scene 管理
- UI 支持切换 scene

验收：
- 一个项目可保存后重开
- 一个项目可包含至少 2 个 scene

---

### Phase 8: 为 OMPL 和碰撞留接入点
- 增加 `ITrajectoryPlanner`
- 增加 `ICollisionChecker`
- 增加 `PlanningSceneSnapshot`
- 先实现默认 planner，OMPL 后续接入

验收：
- 应用层通过接口调用 planner，不感知具体实现

---

## Testing Strategy

### 1. Foundation / Domain 单元测试
验证：
- 值类型行为
- 场景状态合法性
- 项目与场景关系

### 2. ModelImport 测试
验证：
- URDF/机器人导入
- 项目序列化/反序列化
- 路径解析和资源定位

### 3. Geometry 测试
验证：
- STEP 导入成功
- 面引用稳定
- waypoint 生成稳定
- 几何变换正确

### 4. Planning 测试
基于现有 [TestRobotKinematics.cpp](E:/Code/GrindingApp/tests/TestRobotKinematics.cpp) 扩展，验证：
- FK / IK
- IK 多解
- trajectory 插值
- 轨迹错误标记
- 后续碰撞检测接口行为

### 5. Application 集成测试
验证整条业务链：
- 创建项目
- 创建场景
- 加载 robot/tool/workpiece
- 选面
- 生成 waypoint
- 规划 trajectory
- 保存项目
- 重新打开项目

### 6. Qt Adapter 测试
只测：
- UI 是否正确调用应用层
- snapshot 是否正确显示
- 简单交互联动

不在 Qt 层重复测 FK/IK/规划算法。

---

## Acceptance Criteria

当以下条件全部满足时，说明重构达到目标：

- Qt 不是业务状态真源
- `LoadRobot`、`ImportWorkpiece`、`GenerateWaypoints`、`PlanTrajectory` 可以脱离 Qt 运行
- `TrajectoryPlanner` 已从 `ui` 迁出
- `Waypoint` 和 `Trajectory` 已脱离 `occ`
- 一个项目可以包含多个场景
- 项目可以保存并重新打开
- 应用层通过抽象接口调用 geometry / planning
- 后续接入 OMPL 时不需要修改 `MainWindow`
- 后续接入碰撞检测时不需要重做 scene 结构

---

## Assumptions

- 第一阶段允许内部继续使用 OCCT `gp_Trsf` 作为过渡，但 Application 对外接口不应长期直接暴露它
- 当前 `RbRobot` 可作为过渡模型继续使用，但最终要改成格式无关的 `Robot`
- 当前 tests 以运动学为主，重构后要补 Application 和 Persistence 测试
- 不引入 COM；如未来需要跨进程，优先新增本地 RPC 适配层
- 第一阶段继续保留 Qt + OCCT viewer 作为唯一前端实现，但它只是 adapter，不是系统核心
