# GrindingApp 重构实施设计文档（可直接按步骤执行）

## Summary

本方案把当前项目从“Qt UI 主导业务流程”的结构，重构为“后端内核 + Qt 桌面适配器”的结构，并且按可连续落地的顺序拆分。  
目标不是一次推倒重来，而是让你可以在每个阶段都保持项目可编译、可运行、可验证。

本实施方案的核心原则：

- 先建新边界，再迁移旧代码
- 先抽“应用流程”，再抽“数据模型”
- 先做到 Qt 不再直接编排业务，再考虑后续服务化
- 所有重构都围绕“项目/场景/用例服务”展开
- 每一步都给出目标、改动范围、完成标志、风险和验证方式

---

## Final Architecture

最终目标模块为：

1. `Foundation`
2. `Domain`
3. `ModelImport`
4. `Geometry`
5. `Planning`
6. `Application`
7. `QtDesktop`

建议最终目录：

- `src/foundation`
- `src/domain`
- `src/model_import`
- `src/geometry`
- `src/planning`
- `src/application`
- `src/adapters/qt_desktop`

第一阶段可以不立刻改物理目录，但必须先按 CMake target 建立逻辑边界。

---

## Dependency Rules

必须固定依赖方向：

- `SSFoundation` 被所有模块依赖
- `SSDomain` 只依赖 `SSFoundation`
- `SSModelImport` 依赖 `SSFoundation + SSDomain`
- `SSGeometry` 依赖 `SSFoundation + SSDomain`
- `SSPlanning` 依赖 `SSFoundation + SSDomain + SSGeometry` 的稳定输入
- `SSApplication` 依赖 `SSFoundation + SSDomain + SSModelImport/SSGeometry/SSPlanning` 的抽象接口
- `SSQtDesktop` 只依赖 `SSApplication`

禁止出现：

- `SSDomain -> Qt`
- `SSDomain -> OCCT`
- `SSApplication -> QWidget/AIS`
- `SSQtDesktop -> RobotKinematics/StepImporter/WaypointGenerator` 直连
- `Trajectory/Waypoint` 长期留在 `occ` 命名空间

---

## Phase 0：命名和边界冻结

### 目标
在动代码前先统一概念，避免边改边变。

### 决策
采用以下术语：

- `Project`：工程根对象，可包含多个场景
- `Scene`：一个工作站/工艺场景
- `Robot` / `Tool` / `Workpiece`：领域对象
- `WaypointSet`：一组加工路径点
- `Trajectory`：规划结果
- `SceneSnapshot`：前端读取的只读结果
- `SceneAppService`：前端唯一业务入口
- `ProjectAppService`：项目级入口
- `GeometryHandle` / `FaceRef`：稳定几何引用
- `Pose`：统一位姿值类型
- `JointState`：统一关节值类型

### 完成标志
- 不再新增“Controller/Manager/Helper”这类语义模糊的新核心类
- 所有新代码先按上述概念落位

---

## Phase 1：建立新模块骨架，不迁业务

### 目标
先把未来模块壳子搭起来，让后续迁移有落脚点。

### 新建 CMake targets

新增：
- `SSFoundation`
- `SSDomain`
- `SSModelImport`
- `SSGeometry`
- `SSPlanning`
- `SSApplication`
- `SSQtDesktop`

### 当前 target 的过渡策略

现状：
- `GrindingUtils`
- `GrindingCore`
- `GrindingOcc`
- `GrindingKinematics`
- `GrindingUI`

过渡建议：
- 暂时保留旧 target
- 新 target 先空壳创建
- 分阶段迁移文件
- `GrindingApp` 最终改为只链接 `SSQtDesktop`

### 目录骨架

建立目录：
- `src/foundation/`
- `src/domain/`
- `src/model_import/`
- `src/geometry/`
- `src/planning/`
- `src/application/`
- `src/adapters/qt_desktop/`

### 每个模块先创建的最小文件

#### `src/foundation`
- `Result.h`
- `Error.h`
- `Ids.h`
- `Pose.h`
- `JointState.h`

#### `src/domain`
- `Project.h`
- `Scene.h`
- `Robot.h`
- `Tool.h`
- `Workpiece.h`
- `WaypointSet.h`
- `TrajectoryModel.h`

#### `src/model_import`
- `IRobotImporter.h`
- `IToolImporter.h`
- `IProjectSerializer.h`
- `RobotDefinition.h`
- `ToolDefinition.h`
- `ProjectStorageModel.h`

#### `src/geometry`
- `GeometryTypes.h`
- `IGeometryImporter.h`
- `IShapeQueryService.h`
- `IWaypointGenerator.h`

#### `src/planning`
- `IKinematicsService.h`
- `ITrajectoryPlanner.h`
- `ICollisionChecker.h`
- `PlanningTypes.h`

#### `src/application`
- `ProjectAppService.h`
- `SceneAppService.h`
- `ApplicationDtos.h`
- `Snapshots.h`

#### `src/adapters/qt_desktop`
- `QtScenePresenter.h`
- `OccSceneRenderer.h`

### 本阶段不做
- 不迁移旧逻辑
- 不改 UI 行为
- 不改测试逻辑

### 完成标志
- 新 target 全部可编译
- 当前程序行为不变
- 新模块依赖方向正确

### 验证
- CMake 配置通过
- 旧程序仍可启动

---

## Phase 2：先统一基础值类型和错误返回

### 目标
把后续所有跨模块通信的公共语义先定下来。

### 设计

#### `Pose`
先做过渡版：
```cpp
struct Pose {
    double m[16];
};
```

要求：
- 提供从 `gp_Trsf` 转换
- 提供转回 `gp_Trsf` 的适配函数
- 但 `Domain/Application` 头文件里尽量不直接 include `gp_Trsf`

新增：
- `PoseConverters.h/.cpp` 可放 `geometry` 或 `foundation/interop`

#### `JointState`
用当前 `Q`

#### `Result<T>`
```cpp
template <typename T>
class Result;
```

最少包含：
- `bool ok() const`
- `const T& value() const`
- `const Error& error() const`

#### `Error`
```cpp
enum class ErrorCode {
    kInvalidArgument,
    kNotFound,
    kImportFailed,
    kGeometryFailed,
    kPlanningFailed,
    kCollisionDetected,
    kSerializationFailed,
    kInternalError
};
```

### 本阶段迁移
- 新代码一律使用 `Result<T>`
- 新接口一律不返回裸 `bool`

### 完成标志
- 所有新建接口使用 `Pose` / `JointState` / `Result<T>`
- 不再给新模块设计依赖 Qt/OCCT 类型的公共 API

### 验证
- 编译通过
- 无需行为变化

---

## Phase 3：把 `Waypoint` 和 `Trajectory` 迁入 Domain

### 目标
先把最明显放错层的业务对象挪出来。

### 当前文件
- [Waypoint.h](E:/Code/GrindingApp/src/occ/Waypoint.h)
- [Trajectory.h](E:/Code/GrindingApp/src/occ/Trajectory.h)

### 新位置
- `src/domain/WaypointSet.h`
- `src/domain/TrajectoryModel.h`

### 重构动作

#### 1. 新建领域类型
建议拆成：

```cpp
struct Waypoint {
    Pose pose;
    double speed_ratio = 1.0;
};

struct WaypointSet {
    std::string id;
    std::vector<Waypoint> points;
    std::string source_face_token;
    WaypointGenerationConfig generation_config;
};
```

```cpp
enum class MoveType { kMoveJ, kMoveL };
enum class TrajectoryPointStatus { kOk, kIkFailed, kJointJump, kCollision };

struct TrajectoryPoint {
    Pose tcp_pose;
    JointState joint_state;
    MoveType move_type;
    TrajectoryPointStatus status;
    int waypoint_index = -1;
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;
};
```

#### 2. 保留兼容层
短期可在旧头里做别名或转发，避免一次改太多：
- 旧 `occ::Waypoint` 临时转成新类型别名
- 旧 `occ::Trajectory` 临时转发到 domain

但这个兼容层只作为过渡，不要新增逻辑。

#### 3. 调整引用点
要改的主要调用点：
- [MainWindow.h](E:/Code/GrindingApp/src/ui/MainWindow.h)
- [MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp)
- [TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp)

### 完成标志
- `Waypoint` / `Trajectory` 不再定义在 `occ`
- UI、规划、几何都使用 domain 的轨迹和路径点

### 验证
- 主程序可编译
- 轨迹面板显示正常
- waypoint 生成功能不回归

---

## Phase 4：把 `TrajectoryPlanner` 从 UI 抽到 Planning

### 目标
优先抽出最核心的“非 UI 业务逻辑”。

### 当前文件
- [TrajectoryPlanner.h](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.h)
- [TrajectoryPlanner.cpp](E:/Code/GrindingApp/src/ui/TrajectoryPlanner.cpp)

### 新位置
- `src/planning/CartesianTrajectoryPlanner.h`
- `src/planning/CartesianTrajectoryPlanner.cpp`

### 新类名建议
把原 `TrajectoryPlanner` 改成更明确的名字：
- `CartesianTrajectoryPlanner`
或
- `InterpolatingTrajectoryPlanner`

因为它本质是：
- approach pose 计算
- MoveJ 插值
- MoveL 插值
- IK 求解
- joint jump 检查

不是 UI 对象。

### 规划接口
同时建立抽象接口：

```cpp
class ITrajectoryPlanner {
public:
    virtual ~ITrajectoryPlanner() = default;
    virtual Result<Trajectory> Plan(const PlanningSceneSnapshot& scene,
                                    const WaypointSet& waypoints,
                                    const PlanningRequest& request) = 0;
};
```

### 当前实现如何落位
`CartesianTrajectoryPlanner` 实现 `ITrajectoryPlanner`。

### 与旧代码的关系
短期内：
- `MainWindow::OnPlanTrajectory()` 不直接构造 `TrajectoryPlanner`
- 改成通过 `SceneAppService` 触发
- 在 SceneAppService 内部再调用 `ITrajectoryPlanner`

如果 Application 还没建好，可先过渡：
- `MainWindow` 调 `planning::CartesianTrajectoryPlanner`
- 但绝不再放回 UI 模块

### 完成标志
- `TrajectoryPlanner` 不再属于 UI target
- UI 中不包含规划算法代码

### 验证
- 轨迹规划功能与现状一致
- trajectory panel 仍可展示结果

---

## Phase 5：抽出 Kinematics 服务门面

### 目标
把当前自由函数风格的运动学能力，整理成稳定服务接口。

### 当前文件
- [RobotKinematics.h](E:/Code/GrindingApp/src/kinematics/RobotKinematics.h)
- [RobotKinematics.cpp](E:/Code/GrindingApp/src/kinematics/RobotKinematics.cpp)
- [IKinematicsSolver.h](E:/Code/GrindingApp/src/kinematics/IKinematicsSolver.h)
- [KdlSolver.*](E:/Code/GrindingApp/src/kinematics/)
- [KdlChainBuilder.*](E:/Code/GrindingApp/src/kinematics/)

### 新结构建议

#### 抽象接口
`src/planning/IKinematicsService.h`
```cpp
class IKinematicsService {
public:
    virtual ~IKinematicsService() = default;
    virtual Result<std::vector<Pose>> ComputeFk(const Robot& robot,
                                                const JointState& joints) = 0;
    virtual Result<JointState> ComputeIk(const Robot& robot,
                                         const Pose& target,
                                         const JointState& seed) = 0;
    virtual Result<std::vector<JointState>> ComputeIkAll(const Robot& robot,
                                                         const Pose& target,
                                                         const JointState& seed) = 0;
};
```

#### 实现
`KdlKinematicsService`
内部复用现在的 KDL 求解逻辑。

### 当前自由函数如何处理
`ComputeFk/ComputeIk/ComputeIkAllSolutions` 可保留为过渡 facade，但最终由 `KdlKinematicsService` 实现。

### 重要决策
- `Robot` 必须从“解析格式对象”逐步转成“领域机器人模型”
- `IKinematicsService` 不暴露 `gp_Trsf`
- `JointState` 统一用 `values_deg`

### 完成标志
- Application 或 planner 不再直接调用旧自由函数
- 统一通过 `IKinematicsService`

### 验证
继续复用现有测试，特别是：
- [TestRobotKinematics.cpp](E:/Code/GrindingApp/tests/TestRobotKinematics.cpp)

---

## Phase 6：抽出 Robot 导入为 `ModelImport`

### 目标
把“从文件中读机器人”的逻辑从核心和 UI 中移走。

### 当前文件
- [RbXmlParser.h](E:/Code/GrindingApp/src/core/RbXmlParser.h)
- [RbXmlParser.cpp](E:/Code/GrindingApp/src/core/RbXmlParser.cpp)

### 新设计

#### 中间定义对象
`RobotDefinition`
包含：
- 机器人名称
- 关节定义
- 可视 mesh 引用
- 碰撞 mesh 引用
- source path
- 原始元信息

#### 抽象接口
```cpp
class IRobotImporter {
public:
    virtual ~IRobotImporter() = default;
    virtual Result<RobotDefinition> Import(const std::string& path) = 0;
};
```

#### 实现类
- `UrdfRobotImporter`
- 当前 `RbXmlParser` 的实际逻辑并入这里

### 与 Domain 的边界
新增：
- `RobotFactory`
将 `RobotDefinition` 转为 `domain::Robot`

### UI/应用层新流程
旧流程：
- UI -> `RbXmlParser::Parse()`

新流程：
- UI -> `SceneAppService::LoadRobot()`
- `SceneAppService` -> `IRobotImporter`
- `RobotFactory` -> `Scene.AttachRobot(robot)`

### 完成标志
- Qt 不再直接 include `RbXmlParser.h`
- `LoadRobot` 不再由 UI 直接解析文件

### 验证
- 机器人导入后渲染与当前一致
- 现有 robot tests 仍通过

---

## Phase 7：抽出 Workpiece 导入和 Geometry 引用体系

### 目标
给工件、选面、路径点生成建立稳定的后端模型。

### 当前问题
当前 `MainWindow` 长期持有：
- `TopoDS_Shape workpiece_shape_`
- `TopoDS_Face selected_face_`

这会让业务状态绑定到 OCCT 视图细节上。

### 新设计

#### 新类型
```cpp
struct GeometryHandle {
    std::string value;
};

struct FaceRef {
    std::string geometry_id;
    std::string subshape_token;
};
```

#### Geometry importer
```cpp
class IGeometryImporter {
public:
    virtual Result<GeometryAsset> ImportStep(const std::string& path) = 0;
};
```

#### Shape query service
```cpp
class IShapeQueryService {
public:
    virtual Result<std::vector<FaceRef>> EnumerateFaces(const GeometryHandle&) = 0;
    virtual Result<FaceRef> PickFace(const GeometryHandle&, const PickHit& hit) = 0;
};
```

说明：
- Qt Viewport 可以做拾取
- 但拾取结果必须被转换成稳定 `FaceRef`
- `Scene` 只保存 `FaceRef`

#### Workpiece 领域对象
```cpp
struct Workpiece {
    std::string name;
    GeometryHandle geometry;
    Pose pose;
    std::optional<FaceRef> selected_face;
};
```

### 当前文件迁移
- [StepImporter.cpp](E:/Code/GrindingApp/src/occ/StepImporter.cpp)
- [MeshLoader.cpp](E:/Code/GrindingApp/src/occ/MeshLoader.cpp)
- [WaypointGenerator.cpp](E:/Code/GrindingApp/src/occ/WaypointGenerator.cpp)
- [WaypointGridAlgo.cpp](E:/Code/GrindingApp/src/occ/WaypointGridAlgo.cpp)
- [WaypointPlanarAlgo.cpp](E:/Code/GrindingApp/src/occ/WaypointPlanarAlgo.cpp)

### 完成标志
- `Scene` 保存的是 `FaceRef` 而不是 `TopoDS_Face`
- UI 不再把 `TopoDS_Face` 当业务状态真源

### 验证
- 导入工件正常
- 选面正常
- waypoint 生成正常

---

## Phase 8：建立 Domain 的 `Project/Scene` 正式结构

### 目标
从“单窗口状态堆”升级到“项目 + 多场景”。

### 新对象设计

#### `Project`
```cpp
class Project {
public:
    Project(ProjectId id, std::string name);
    SceneId AddScene(const std::string& name);
    bool RemoveScene(SceneId id);
    Scene* FindScene(SceneId id);
    const std::vector<Scene>& Scenes() const;
};
```

#### `Scene`
```cpp
class Scene {
public:
    Scene(SceneId id, std::string name);

    Result<void> AttachRobot(Robot robot);
    Result<void> AttachTool(Tool tool);
    Result<void> AttachWorkpiece(Workpiece workpiece);

    Result<void> SetRobotBasePose(const Pose&);
    Result<void> SetWorkpiecePose(const Pose&);
    Result<void> SetJointState(const JointState&);
    Result<void> SetSelectedFace(const FaceRef&);
    Result<void> SetWaypoints(WaypointSet);
    Result<void> SetTrajectory(Trajectory);

    const Robot* GetRobot() const;
    const Tool* GetTool() const;
    const Workpiece* GetWorkpiece() const;
};
```

### Scene 中建议持有的状态
- robot
- tool
- workpiece
- robot base pose
- current joint state
- current tcp pose cache 可选
- selected face ref
- current waypoint set
- current trajectory
- planning profile
- collision profile

### 当前 MainWindow 上的这些状态，要逐步迁入 Scene
- `waypoints_`
- `trajectory_`
- `workpiece_trsf_`
- `selected_face_`
- 机器人关节角
- tool tcp/base 关系状态

### 完成标志
- 主状态来源是 `Scene`
- UI 内部不再是唯一真源

### 验证
- 逻辑上允许创建多 scene
- 即使 Qt 暂时只显示一个 active scene，后台结构也已支持多个

---

## Phase 9：建立 Application 服务，替换 UI 直连业务

### 目标
这是整个重构的核心阶段。

### 新增服务

#### `ProjectAppService`
负责：
- 创建项目
- 打开项目
- 保存项目
- 创建场景
- 切换活动场景
- 获取项目快照

#### `SceneAppService`
负责：
- `LoadRobot`
- `LoadTool`
- `ImportWorkpiece`
- `SelectFace`
- `GenerateWaypoints`
- `PlanTrajectory`
- `SetRobotBasePose`
- `SetWorkpiecePose`
- `SetJointState`
- `SetTcpPose`
- `ClearTrajectory`

### SceneAppService 内部依赖
- `IRobotImporter`
- `IToolImporter`
- `IGeometryImporter`
- `IShapeQueryService`
- `IWaypointGenerator`
- `IKinematicsService`
- `ITrajectoryPlanner`

### SceneAppService 的内部状态管理
建议持有：
- `ProjectRepository` 或当前活动 `Project`
- `SceneSnapshotBuilder`

### 关键 DTO 设计

#### `LoadRobotRequest`
```cpp
struct LoadRobotRequest {
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

### SceneSnapshot 必须覆盖
- scene id / name
- robot summary
- tool summary
- workpiece summary
- joints
- tcp pose
- selected face ref
- waypoint count / preview info
- trajectory status / error count
- object transforms

### UI 替换策略
旧：
- `MainWindow` 直接调用导入、规划、选面逻辑

新：
- `MainWindow` 只调用 `SceneAppService`
- 然后消费 `SceneSnapshot`

### 完成标志
- `MainWindow` 不再调用：
  - `StepImporter`
  - `WaypointGenerator`
  - `ComputeIk`
  - `TrajectoryPlanner`

### 验证
- 现有主流程全部仍能跑
- UI 行为与现在一致或更稳定

---

## Phase 10：拆分 `RobotController`

### 目标
把当前最大的混合职责类拆开。

### 当前职责混杂
[RobotController.h](E:/Code/GrindingApp/src/ui/RobotController.h) 现在同时做了：
- 机器人导入
- 工具导入
- 业务状态维护
- FK 计算
- OCC 渲染
- Qt 信号转发

必须拆开。

### 新拆分方案

#### A. `SceneAppService`
承接：
- 机器人和工具加载
- 关节更新
- base pose 更新
- tcp pose 求解
- 业务状态维护

#### B. `OccSceneRenderer`
承接：
- AIS 对象创建和销毁
- robot/tool/workpiece 显示
- 坐标系显示
- 根据 `SceneSnapshot + geometry assets` 刷新视图

#### C. `QtScenePresenter`
承接：
- scene tree
- labels
- panel 同步
- IK solutions list 同步

### 删除思路
最终 `RobotController` 不应继续作为核心类存在。  
可以保留一个短期兼容壳，但长期应删除。

### 完成标志
- Qt 层没有单一“控制所有业务 + 渲染”的大类
- 渲染与业务状态分离

### 验证
- 修改 joint/base/workpiece pose 后，视图仍同步刷新
- scene tree 和 tcp display 正常

---

## Phase 11：改写 MainWindow 为纯适配器

### 目标
让 `MainWindow` 只剩 UI 行为，不再承载业务逻辑。

### `MainWindow` 最终应该保留的职责
- 菜单和工具栏
- 文件选择框
- 对 panel 的连接
- 把用户操作转发给 app service
- 将 app service 返回的 snapshot 交给 presenter / renderer

### `MainWindow` 中必须迁出的逻辑
从 [MainWindow.cpp](E:/Code/GrindingApp/src/ui/MainWindow.cpp) 迁出：
- `OnImportWorkpiece` 内的导入和状态清理逻辑
- `OnGenerateWaypoints`
- `OnPlanTrajectory`
- `OnPoseEdited` 内 IK 求解逻辑
- `TransformWaypointsAndTrajectory`
- `ReplanTrajectory`

其中：
- 这些函数名可以保留为 UI 事件入口
- 但函数体只应做参数整理和 service 调用

### 示例目标形态

旧：
```cpp
void MainWindow::OnPlanTrajectory(double approach_dist) {
    // UI 内直接做 planning
}
```

新：
```cpp
void MainWindow::OnPlanTrajectory(double approach_dist) {
    PlanTrajectoryRequest req = ...;
    auto result = scene_service_->PlanTrajectory(req);
    if (!result.ok()) { ... }
    ApplySceneSnapshot(result.value());
}
```

### 完成标志
- `MainWindow` 中没有算法性代码
- `MainWindow` 中没有 domain 真状态

### 验证
- 主窗口代码行数明显下降
- UI 仅做调度和显示

---

## Phase 12：项目保存/打开

### 目标
建立长期可演进的项目持久化。

### 新组件
- `IProjectSerializer`
- `JsonProjectSerializer` 或 `TomlProjectSerializer`
- `ProjectStorageModel`

### 建议文件格式内容
根级：
- project name
- version
- resource roots
- scenes[]

每个 scene 保存：
- robot resource ref
- tool resource ref
- workpiece resource ref
- robot base pose
- workpiece pose
- current joint state
- selected face ref
- waypoints
- trajectory
- planning config
- collision config placeholder

### 路径策略
建议：
- 项目文件相对路径优先
- 内部保存原始资源路径和规范化相对路径
- 加版本号字段，便于以后升级格式

### 打开流程
1. serializer 解析项目文件
2. 建立 `ProjectStorageModel`
3. 通过 app service 恢复 `Project`
4. 重新加载资源与 geometry handles
5. 返回 `ProjectSnapshot`

### 保存流程
1. 从当前 `Project` 构建 storage model
2. serializer 输出文件

### 完成标志
- 可以保存并重开一个完整场景
- 重开后依然能继续规划和编辑

### 验证
- round-trip 测试：open -> save -> reopen -> compare snapshot

---

## Phase 13：多场景支持

### 目标
让后续“其他场景”不是特殊功能，而是内建能力。

### Application 增加接口
- `CreateScene`
- `RemoveScene`
- `CloneScene`
- `SetActiveScene`
- `ListScenes`

### UI 第一阶段要求
即使还没有完整 tab UI，也要至少支持：
- 当前工程里有多个 scene
- scene tree 或菜单中能切换 active scene

### Scene 间隔离要求
每个 scene 独立持有：
- robot/tool/workpiece
- transforms
- waypoints
- trajectory
- planner config

禁止多个 scene 默认共享可变运行时状态。

### 完成标志
- 项目文件可保存多个 scene
- 切换 scene 时视图和 panel 正确刷新

---

## Phase 14：为 OMPL 和碰撞留接口但不强制实现

### 目标
这阶段主要是“架构占位”，不是全部实现功能。

### 需要新增的接口

#### `PlanningSceneSnapshot`
包含：
- robot
- current joint state
- tool
- workpiece
- object transforms
- geometry refs / collision geometry refs
- constraints
- planner config

#### `ICollisionChecker`
```cpp
class ICollisionChecker {
public:
    virtual Result<CollisionReport> CheckScene(const PlanningSceneSnapshot&) = 0;
    virtual Result<CollisionReport> CheckTrajectory(const PlanningSceneSnapshot&,
                                                    const Trajectory&) = 0;
};
```

#### `ITrajectoryPlanner`
当前已有，未来 OMPL 实现它。

### 当前默认实现
- `CartesianTrajectoryPlanner` 继续作为默认 planner
- `NoOpCollisionChecker` 可做空实现

### 这样做的原因
以后接入 OMPL 时：
- 不改 UI
- 不改 app service 接口
- 只新增 planner 实现与配置项

### 完成标志
- `SceneAppService` 不依赖具体 planner 类名
- planner/collision 通过注入接口使用

---

## Concrete File-by-File Migration Order

### 第 1 批：低风险、收益高
1. 建新目录和新 targets
2. 新建 `Result/Error/Ids/Pose/JointState`
3. 迁 `Waypoint/Trajectory` 到 domain
4. 迁 `TrajectoryPlanner` 到 planning

### 第 2 批：核心能力抽象
5. 建 `IKinematicsService`
6. 封装 KDL 实现
7. 建 `IRobotImporter` 和 `RobotDefinition`
8. 包装现有 `RbXmlParser`

### 第 3 批：应用层骨架
9. 建 `Project/Scene`
10. 建 `SceneSnapshot`
11. 建 `SceneAppService`
12. 先把 `LoadRobot/PlanTrajectory` 接进去

### 第 4 批：几何与工件链路
13. 建 `GeometryHandle/FaceRef`
14. 抽 `IGeometryImporter`
15. 抽 `IWaypointGenerator`
16. 接 `ImportWorkpiece/GenerateWaypoints`

### 第 5 批：Qt 退化为适配层
17. `MainWindow` 改为调用 `SceneAppService`
18. 拆 `RobotController`
19. 建 `OccSceneRenderer`
20. 建 `QtScenePresenter`

### 第 6 批：项目系统
21. 建 `ProjectAppService`
22. 建 serializer
23. 接保存/打开
24. 接多场景切换

### 第 7 批：扩展能力占位
25. 建 `PlanningSceneSnapshot`
26. 建 `ICollisionChecker`
27. 接 planner injection
28. 预留 OMPL implementation slot

---

## Tests You Should Add In Parallel

### A. 新增 domain 测试
- `Project` 增删 scene
- `Scene` attach robot/tool/workpiece
- `Scene` 状态变更合法性

### B. 新增 application 集成测试
至少覆盖：
- `LoadRobot`
- `ImportWorkpiece`
- `GenerateWaypoints`
- `PlanTrajectory`
- `SaveProject/OpenProject`

### C. 扩展现有 planning 测试
在 [TestRobotKinematics.cpp](E:/Code/GrindingApp/tests/TestRobotKinematics.cpp) 基础上补：
- planner output point count
- joint jump 标记
- trajectory round-trip

### D. 几何测试
- STEP 导入
- face ref 稳定性
- waypoint generation consistency

### E. 项目序列化测试
- scene 保存再恢复
- 多 scene 保存再恢复
- 相对路径解析

---

## Milestone Definition

### Milestone 1：模块壳子完成
满足：
- 新 targets 已建立
- 新值类型已建立
- 项目还能编译

### Milestone 2：规划脱离 UI
满足：
- `TrajectoryPlanner` 已迁出 UI
- `Waypoint/Trajectory` 已进入 domain

### Milestone 3：机器人和工件主流程进入 application
满足：
- `LoadRobot`
- `ImportWorkpiece`
- `GenerateWaypoints`
- `PlanTrajectory`
全部通过 `SceneAppService`

### Milestone 4：Qt 成为纯适配层
满足：
- `MainWindow` 不直接调用底层规划/导入能力
- `RobotController` 被拆解

### Milestone 5：项目系统成立
满足：
- 支持保存/打开
- 支持多场景

### Milestone 6：扩展架构完成
满足：
- planner injection
- collision interface
- OMPL 占位接入点

---

## Practical Execution Notes

### 1. 不要一次性全改 include
建议每阶段用兼容头/别名过渡，等调用点都迁完再删旧头。

### 2. 不要先大改目录再改逻辑
推荐顺序是：
- 先新建目标模块
- 迁类
- 改依赖
- 最后再考虑物理目录整理

### 3. 保持程序始终可运行
每个 phase 最好都满足：
- 编译通过
- UI 可启动
- 至少基本流程可走

### 4. 优先把“流程逻辑”迁出，而不是先做完美抽象
比起先设计 30 个接口，更重要的是先把 `MainWindow` 里的业务拿出来。

### 5. 不要过早引入服务化、COM、消息总线
当前阶段全部用同步接口 + snapshot 就够了。  
跨进程和外部 API 是下一层适配，不是这次重构核心。

---

## Acceptance Criteria For The Whole Refactor

重构完成后，以下条件必须成立：

- 不启动 Qt 也能跑导入、路径生成、轨迹规划、保存/打开
- `MainWindow` 不再是业务真源
- `Scene` 成为运行时核心状态容器
- `SceneAppService` 成为前端唯一业务入口
- `TrajectoryPlanner` 已不在 UI
- `Waypoint/Trajectory` 已不在 `occ`
- 机器人导入逻辑不再由 UI 直接调用
- 工件选面状态保存为稳定 `FaceRef`
- 项目支持多 scene
- 后续接入 OMPL 只需新增 planner 实现
- 后续接入碰撞检测只需实现 `ICollisionChecker` 并接入 planning scene

---

## Assumptions

- 第一阶段允许内部仍有 `gp_Trsf` 适配代码，但公共边界不继续扩散它
- `RbRobot` 可作为过渡对象保留一段时间，但最终会被 `domain::Robot` 替代
- 现有 Qt 前端先继续保留
- 当前最优重构节奏是渐进式，而不是分支上一次性重写
- 不引入 COM，未来如需跨进程优先做本地 RPC 适配层
