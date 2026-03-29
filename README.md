# GrindingApp

GrindingApp 是一个基于 Qt + OpenCASCADE + KDL 的打磨工作站原型，当前主流程已经可以完成：

- 导入机器人
- 导入工具
- 导入工件
- 选取加工面
- 生成打磨路径点
- 做逆解与轨迹规划
- 回放基础流程

当前里程碑标签：

- `milestone/basic-flow-runnable`
- `v0.1-basic-flow`

这两个标签对应“导入机器人、工具、工件，选面，生成打磨点，逆解成功，基础流程可跑通”的状态。

## 当前技术路线

项目目前围绕下面这条链路组织：

- 机器人模型：`URDF`
- 工具模型：`URDF`
- 工件模型：`STEP`
- 正逆运动学：`urdfdom + Orocos KDL`
- 显示：`OpenCASCADE`

机器人不再依赖旧的 `.rb.xml` 作为主运动学来源，工具也已经切到 `URDF-only`。

## 已跑通的基础流程

UI 里的主流程是：

1. `File -> Load Robot`
2. `File -> Load Tool`
3. `File -> Import Workpiece`
4. `Path -> Select Face`
5. `Path -> Generate Waypoints`
6. `Trajectory Editor -> Plan`
7. 在 Jog / Trajectory / Playback 面板里检查、切换 IK 解和回放

当前仓库内可直接参考的模型：

- 机器人
  - [IRB140.urdf](model/robot/IRB140/IRB140.urdf)
  - [irb2400.urdf](model/robot/irb2400/irb2400.urdf)
- 工具
  - [Burr.urdf](model/tool/Burr/Burr.urdf)
  - [HQ2.urdf](model/tool/HQ2/HQ2.urdf)
- 工件
  - [box.stp](model/workpiece/box.stp)

## 模型和单位约定

这部分是这段时间改动最多、也最容易踩坑的地方。

### 1. 机器人和工具的 URDF 角度约定

为了让模型文件更好读，项目现在约定：

- `URDF` 里的 `origin rpy` 用“度”写
- 机器人的转动关节 `limit lower/upper/velocity` 也用“度”写

但 `urdfdom` 和 `KDL` 运行时仍然要求弧度，所以代码里会在进入运动学前做一次转换：

- 机器人：在 [KdlChainBuilder.cpp](src/kinematics/KdlChainBuilder.cpp) 中转成弧度后再交给 `urdfdom/KDL`
- 工具：在 [RobotController.cpp](src/ui/RobotController.cpp) 中按“度”直接读入显示变换

### 2. 运行时场景长度单位

运行时场景现在统一按“米”处理。

这意味着：

- 机器人 `URDF` 按米参与 KDL 和显示
- 工具 `URDF` 通过 `mesh scale` 进入米制场景
- 工件 `STEP` 导入时会在 [StepImporter.cpp](src/occ/StepImporter.cpp) 里从毫米缩放到米

### 3. UI 显示单位

虽然运行时场景已经统一到米，但操作界面依旧尽量对操作者保持熟悉的输入方式：

- Jog 面板 TCP 平移显示为 `mm`
- Move 面板平移显示为 `mm`
- 旋转角度显示为 `deg`

对应的 `mm <-> m` 转换现在集中放在：

- [MainWindow.cpp](src/ui/MainWindow.cpp)
- [MovementPanel.cpp](src/ui/MovementPanel.cpp)
- [RobotController.cpp](src/ui/RobotController.cpp)

## 关键代码入口

如果你要继续维护这条链，建议优先看这些文件：

- 机器人模型解析
  - [RbXmlParser.cpp](src/core/RbXmlParser.cpp)
- URDF 到 KDL 建链
  - [KdlChainBuilder.cpp](src/kinematics/KdlChainBuilder.cpp)
- KDL 正逆解入口
  - [KdlSolver.cpp](src/kinematics/KdlSolver.cpp)
  - [RobotKinematics.cpp](src/kinematics/RobotKinematics.cpp)
- 机器人/工具显示与坐标系
  - [RobotController.cpp](src/ui/RobotController.cpp)
  - [RobotDisplay.cpp](src/occ/RobotDisplay.cpp)
- 工件导入
  - [StepImporter.cpp](src/occ/StepImporter.cpp)
- 主 UI 流程
  - [MainWindow.cpp](src/ui/MainWindow.cpp)

## 构建依赖

顶层 [CMakeLists.txt](CMakeLists.txt) 当前依赖：

- `Qt5`
- `OpenCASCADE`
- `Eigen3`
- `orocos-kdl`
- `assimp`
- `urdfdom`

`vcpkg.json` 已经纳入项目依赖管理，推荐通过 `vcpkg + CMake` 构建。

## 构建方式

常见方式：

```powershell
cmake -S . -B build
cmake --build build --config Release
```

可执行文件通常在：

```text
build/bin/Release/GrindingApp.exe
```

## 当前注意事项

- 当前根目录之外的 [user-guide.md](docs/user-guide.md) 有编码问题，阅读体验不如这份 `README`
- 本地这台开发机此前出现过 `CMake / VS / vcpkg` 构建链不稳定的问题，所以某些最近改动虽然已经提交，但未必都在当前机器上重新完整编译验证过
- 现阶段优先保证“基础流程可跑通”，后续再继续收敛显示细节、测试覆盖和更完整的模型导入能力

## 后续建议

如果接下来继续推进，比较值得优先做的是：

1. 修复并更新 `docs/user-guide.md` 的编码和内容
2. 恢复稳定的本机构建/测试链
3. 给 `URDF -> KDL`、工具导入和工件单位归一化补更明确的自动化测试
4. 继续补充 `DAE/visual mesh` 渲染链，减少当前对 `STL/collision` 的依赖
