# Phase 6 设计文档：抽出 Robot 导入为 ModelImport 层

**日期**：2026-04-12  
**状态**：待实施

---

## 问题背景

Phase 5 完成了 `domain::Robot`（纯运动学模型）和 `IKinematicsService`，但遗留了以下问题：

| 问题 | 根源 |
|---|---|
| `RobotController` 硬编码调用 `RbXmlParser::Parse()` | 导入格式与渲染控制耦合在同一个类 |
| 3 处重复的临时 `ToDomainRobot()` 桥接函数 | `current_robot_` 存的是 `RbRobot`，但 KDL/IK 需要 `domain::Robot` |
| `RbRobot` 把运动学数据和渲染数据混在一起 | 历史遗留结构，Phase 5 未迁移 |
| `RobotDisplay.h` 对外暴露 `RbRobot` 参数 | `GrindingOcc` 依赖 `GrindingCore` 数据结构 |

---

## 目标

1. 填充 `SSModelImport` 接口层（4 个空头文件）
2. 在 `GrindingCore` 实现 `UrdfRobotImporter` 和 `UrdfToolImporter`
3. `RobotController` 通过接口导入，持有 `domain::Robot` 而非 `RbRobot`
4. 删除所有 `ToDomainRobot()` 临时桥接

---

## 数据结构

### `src/model_import/RobotDefinition.h`

```cpp
namespace model_import {

struct MeshRef {
    std::string name;
    std::string ref_joint;        // "Robot_Base" 或 "Joint1".."Joint6"
    nl::utils::Vector3d rpy;      // degrees: yaw, pitch, roll
    nl::utils::Vector3d pos;      // mm
    std::string mesh_file;        // absolute path
};

struct RobotDefinition {
    domain::Robot model;                  // 纯运动学（无渲染数据）
    std::vector<MeshRef> meshes;          // 渲染用网格引用
};

} // namespace model_import
```

### `src/model_import/ToolDefinition.h`

```cpp
namespace model_import {

struct ToolDefinition {
    std::string name;
    std::string source_path;
    std::string mesh_file;               // absolute path
    nl::utils::Vector3d mesh_scale;      // (1,1,1) if no scale attribute
    nl::utils::Vector3d base_rpy_deg;    // yaw, pitch, roll
    nl::utils::Vector3d base_pos_mm;
    nl::utils::Vector3d tcp_rpy_deg;
    nl::utils::Vector3d tcp_pos_mm;
};

} // namespace model_import
```

---

## 接口

```cpp
// src/model_import/IRobotImporter.h
namespace model_import {
class IRobotImporter {
public:
    virtual ~IRobotImporter() = default;
    virtual foundation::Result<RobotDefinition> Import(const std::string& file_path) = 0;
};
} // namespace model_import

// src/model_import/IToolImporter.h
namespace model_import {
class IToolImporter {
public:
    virtual ~IToolImporter() = default;
    virtual foundation::Result<ToolDefinition> Import(const std::string& file_path) = 0;
};
} // namespace model_import
```

---

## 实现层（GrindingCore）

`UrdfRobotImporter::Import()` 内部流程：
```
RbXmlParser::Parse(file_path) → RbRobot
  ↓ ToRobotDefinition()（匿名 namespace 内）
RobotDefinition { model: domain::Robot, meshes: vector<MeshRef> }
  ↓ Result::Ok()
foundation::Result<RobotDefinition>
```

`UrdfToolImporter::Import()` 内部流程：
- 迁移 `RobotController.cpp` 中的 `ParseToolUrdf()` 及所有 helper 函数
- 返回 `foundation::Result<ToolDefinition>`

---

## RobotController 变化

### 构造函数（注入导入器）

```cpp
// 改前
explicit RobotController(Handle(AIS_InteractiveContext) context, QObject* parent = nullptr);

// 改后
explicit RobotController(
    Handle(AIS_InteractiveContext) context,
    std::unique_ptr<model_import::IRobotImporter> robot_importer,
    std::unique_ptr<model_import::IToolImporter> tool_importer,
    QObject* parent = nullptr);
```

### 关键类型变化

| 成员/返回值 | 改前 | 改后 |
|---|---|---|
| `current_robot_` | `nl::core::RbRobot` | `domain::Robot` |
| `RobotMesh::drawable` | `nl::core::RbDrawable` | `model_import::MeshRef` |
| `GetRobot()` | `const RbRobot&` | `const domain::Robot&` |
| `LoadRobot()` 内部 | `RbXmlParser::Parse()` | `robot_importer_->Import()` |
| `UpdateRobotDisplay()` | `ToDomainRobot(current_robot_)` | `current_robot_`（直接传） |

### MainWindow 构造注入

```cpp
controller_ = new RobotController(
    context_,
    std::make_unique<nl::core::UrdfRobotImporter>(),
    std::make_unique<nl::core::UrdfToolImporter>(),
    this);
```

---

## 删除范围（完成后应从代码库中消失）

- `RobotController.cpp` 中的 `ToDomainRobot()` 及 `ParseToolUrdf()` + 所有 helper 静态函数（`ParseXyz`, `ParseUrdfRpyDegrees`, `ParseScaleTriple`, `ResolveRelativePath`, `FindUrdfRootLink`, `FindUrdfLinkByName`）
- `MainWindow.cpp` 中的 `ToDomainRobot()`
- `RobotDisplay.cpp` 中的 `ToDomainRobot()` + `using RbRobot/RbJoint`
- `RobotController.h` 中的 `#include "RbXmlParser.h"`
- `RobotDisplay.h` 中的 `#include "RbXmlParser.h"`

---

## 执行步骤（按编译安全顺序）

1. 填充 `SSModelImport` 4 个头文件
2. 新建 `UrdfRobotImporter.h/cpp` + `UrdfToolImporter.h/cpp`
3. 更新 `src/core/CMakeLists.txt`（新文件 + `SSModelImport` 链接）
4. **编译确认 GrindingCore 通过**
5. 迁移 `RobotController.h/cpp`
6. 迁移 `MainWindow.cpp`
7. 更新 `RobotDisplay.h/cpp`
8. 更新 `src/ui/CMakeLists.txt`
9. **编译确认全部通过**
10. 写单元测试
11. 更新 `architecture.md`
12. **编译 + 测试全部通过**

---

## 验证

```bash
cmake --build build --config Release
cd build && ctest --output-on-failure -C Release
```

功能验证：
- 加载 robot URDF → 3D 视图正常显示
- 拖动关节滑条 → 机器人跟随运动
- 加载 tool URDF → 工具末端正常附着
- IK 求解 → 机器人跳到目标位置
- 搜索 `ToDomainRobot` → 0 结果
- 搜索 `RbRobot` → 仅剩 `RbXmlParser.h/cpp` + `UrdfRobotImporter.cpp`

---

## 设计模式说明

### Strategy（`IRobotImporter`）
- **何时用**：调用方需要"导入文件"这个能力，但不应绑定具体格式
- **本次怎么用**：`RobotController` 依赖 `IRobotImporter` 接口；`UrdfRobotImporter` 是当前唯一实现
- **好处**：新增格式只需新增实现类，调用方零修改
- **缺点**：当前只有一种格式，接口略显超前；小项目可能过度设计
- **识别信号**：`class IXxxImporter { virtual Result<T> Import(...) = 0; }`

### Facade（`UrdfRobotImporter::Import()`）
- **何时用**：对外暴露一个简单入口，内部封装多步流程
- **本次怎么用**：`Import()` 内部封装 XML 解析 → 结构转换 → Result 包装
- **好处**：调用方只关心"给路径，得定义"，不关心 DOM/转换细节
- **缺点**：隐藏细节，调试时需深入看内部

### 构造函数注入（Dependency Injection）
- **何时用**：调用方需要可替换的依赖（尤其为了测试）
- **本次怎么用**：`RobotController` 在构造时接收导入器，由 `MainWindow` 注入具体实现
- **好处**：测试时可注入 Mock，生产时注入 Urdf 实现
- **缺点**：构造函数参数增多；QObject 子类需注意所有权
