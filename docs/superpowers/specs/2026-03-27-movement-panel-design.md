# Robot Base & Workpiece Movement — Design Document

## Context

GrindingApp 当前机器人固定在世界原点，工件在 STEP 导入坐标下无法移动。为支持实际磨削仿真场景，需要添加：
1. **Robot Base Movement** — 6-DOF 移动机器人基座
2. **Workpiece Movement** — 6-DOF 移动工件

---

## User Interaction Flow

```
用户右键场景树中的 Robot/Workpiece
  → 右键菜单 → "移动"
    → 弹出临时 Dock 面板 (MovementPanel)
      ┌─ 移动: IRB140 ──────────────────┐
      │  X: [ 100.000] mm               │
      │  Y: [   0.000] mm               │
      │  Z: [ 200.000] mm               │
      │  RX: [  0.000] °                │
      │  RY: [  0.000] °                │
      │  RZ: [ 90.000] °                │
      │                                  │
      │      [确定]    [取消]            │
      └──────────────────────────────────┘
    → spinbox 失去焦点 → 3D 自动预览
    → 点"确定" → 提交 (记入 undo 栈), 面板销毁
    → 点"取消" → 恢复原位, 面板销毁
```

**关键交互规则：**
- 面板是临时的：点"移动"时 new，确定/取消时 delete
- Robot 和 Workpiece 共用同一个面板类，标题显示当前编辑对象名
- 初始值 = 物体当前位置 (首次为 0,0,0,0,0,0)
- 单位：位移 mm, 旋转 degree (RPY)
- 预览阶段不产生 undo 记录，仅确定后一次性记录
- 架构预留鼠标拖拽扩展接口

---

## Impact on Existing Features

| 事件 | Waypoints | Trajectory |
|------|-----------|------------|
| 工件移动 | 跟着变换 | 跟着变换 |
| 机器人 base 移动 | 不变（贴在工件上） | 自动重新规划（IK 重算） |

---

## Architecture

### Transform Pipeline (修改后)

```
Robot mesh:   world = base_trsf_ * fk[joint] * local_transform
Tool:         world = base_trsf_ * fk[6] * tool_base_trsf_
TCP:          world = base_trsf_ * fk[6] * tool_tcp_trsf_
Workpiece:    context_->SetLocation(workpiece_ais_, TopLoc_Location(workpiece_trsf_))
IK target:    target_base = base_trsf_.Inverted() * target_world
```

### Data Ownership

| 数据 | 所在类 | 理由 |
|------|--------|------|
| `base_trsf_` | `RobotController` | 机器人显示逻辑已封装于此 |
| `workpiece_trsf_` | `MainWindow` | 工件显示由 MainWindow 管理 |

---

## New File: MovementPanel

```
src/ui/MovementPanel.h/.cpp  (新增)
```

临时 dock 面板，robot/workpiece 共用：

```cpp
// MovementPanel.h
#ifndef GRINDINGAPP_SRC_UI_MOVEMENT_PANEL_H_
#define GRINDINGAPP_SRC_UI_MOVEMENT_PANEL_H_

#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <gp_Trsf.hxx>

#include "GrindingUIExport.h"

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT MovementPanel : public QDockWidget {
    Q_OBJECT

public:
    explicit MovementPanel(const QString& target_name,
                           const gp_Trsf& initial_trsf,
                           QWidget* parent = nullptr);

    gp_Trsf GetInitialTrsf() const { return initial_trsf_; }

signals:
    void PreviewRequested(const gp_Trsf& trsf);  // spinbox 失去焦点时
    void Accepted(const gp_Trsf& trsf);           // 点确定
    void Cancelled();                               // 点取消

private:
    void SetupUi(const QString& target_name);
    void OnSpinBoxEditingFinished();  // 任一 spinbox 失焦 → emit PreviewRequested
    gp_Trsf ComputeTrsf() const;     // 从 6 个 spinbox 值构建 gp_Trsf

    QDoubleSpinBox* spinboxes_[6] = {};  // X Y Z RX RY RZ
    QPushButton*    ok_btn_ = nullptr;
    QPushButton*    cancel_btn_ = nullptr;

    gp_Trsf initial_trsf_;  // 用于取消时恢复
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_MOVEMENT_PANEL_H_
```

**扩展预留：** 未来添加鼠标拖拽时，外部直接调用 `PreviewRequested` 信号即可，面板架构不变。

---

## Modified Files

| File | Changes |
|------|---------|
| `src/ui/RobotController.h` | + `base_trsf_`, `SetBaseTrsf()`, `GetBaseTrsf()`, `BaseTrsfChanged` signal |
| `src/ui/RobotController.cpp` | `UpdateRobotDisplay()` / `UpdateCoordinateFrames()` 前乘 `base_trsf_` |
| `src/ui/Commands.h` | + `CmdMoveBase`, `CmdMoveWorkpiece` |
| `src/ui/Commands.cpp` | 实现 undo/redo (每次确定后一次性记录) |
| `src/ui/MainWindow.h` | + `workpiece_trsf_`, `SetWorkpieceTrsf()`, `movement_panel_` 指针, 新 slots |
| `src/ui/MainWindow.cpp` | 右键菜单 "移动"、面板创建/销毁、信号接线、坐标变换 |
| `src/ui/CMakeLists.txt` | + MovementPanel.h/.cpp |
| `architecture.md` | 更新 |

### New Files

| File | Description |
|------|-------------|
| `src/ui/MovementPanel.h` | 临时移动面板 header |
| `src/ui/MovementPanel.cpp` | 临时移动面板实现 |

---

## Command Classes

```cpp
// Commands.h — ID: CmdJogJoints=1, CmdMoveBase=2, CmdMoveWorkpiece=3

class CmdMoveBase : public QUndoCommand {
public:
    CmdMoveBase(RobotController* ctrl,
                const gp_Trsf& old_trsf, const gp_Trsf& new_trsf,
                QUndoCommand* parent = nullptr);
    void undo() override;  // ctrl->SetBaseTrsf(old_trsf_)
    void redo() override;  // ctrl->SetBaseTrsf(new_trsf_)
    int id() const override { return 2; }
    // 不需要 mergeWith：确定后一次性提交，不存在连续合并
private:
    RobotController* controller_;
    gp_Trsf old_trsf_, new_trsf_;
};

class CmdMoveWorkpiece : public QUndoCommand {
public:
    using ApplyFn = std::function<void(const gp_Trsf&)>;
    CmdMoveWorkpiece(ApplyFn fn,
                     const gp_Trsf& old_trsf, const gp_Trsf& new_trsf,
                     QUndoCommand* parent = nullptr);
    void undo() override;  // apply_fn_(old_trsf_)
    void redo() override;  // apply_fn_(new_trsf_)
    int id() const override { return 3; }
private:
    ApplyFn apply_fn_;     // 避免 MainWindow 循环依赖
    gp_Trsf old_trsf_, new_trsf_;
};
```

注意：不需要 `mergeWith`，因为用户每次确定才产生一条命令，不像 slider 连续拖拽。

---

## RobotController Changes

```cpp
// RobotController.h 新增
private:
    gp_Trsf base_trsf_;  // 默认 identity

public:
    void SetBaseTrsf(const gp_Trsf& trsf);
    gp_Trsf GetBaseTrsf() const { return base_trsf_; }

signals:
    void BaseTrsfChanged(const gp_Trsf& trsf);
```

### UpdateRobotDisplay() 修改

```cpp
// 当前: world = fk[idx] * local
// 修改为:
gp_Trsf world = base_trsf_;
world.Multiply(fk[idx]);
world.Multiply(local);
context_->SetLocation(m.ais, TopLoc_Location(world));

// Tool:
gp_Trsf tool_world = base_trsf_;
tool_world.Multiply(fk.back());
tool_world.Multiply(tool_base_trsf_);
```

### UpdateCoordinateFrames() 修改

```cpp
base_frame_ = MakeTrihedron(base_trsf_, 80.0);

// Joint frames:
gp_Trsf joint_world = base_trsf_;
joint_world.Multiply(fk[i]);
```

---

## MainWindow Changes

### New Members

```cpp
// MainWindow.h
private:
    gp_Trsf workpiece_trsf_;
    MovementPanel* movement_panel_ = nullptr;

    // 当前编辑目标
    enum class MoveTarget { kNone, kRobot, kWorkpiece };
    MoveTarget move_target_ = MoveTarget::kNone;

    void SetWorkpieceTrsf(const gp_Trsf& trsf);
    void OnMoveRobot();          // 右键菜单触发
    void OnMoveWorkpiece();      // 右键菜单触发
    void OnMovePreview(const gp_Trsf& trsf);   // 预览
    void OnMoveAccepted(const gp_Trsf& trsf);  // 确定
    void OnMoveCancelled();                      // 取消
    void TransformWaypointsAndTrajectory(const gp_Trsf& old_trsf, const gp_Trsf& new_trsf);
    void ReplanTrajectory();
```

### Right-Click Menu (OnSceneTreeContextMenu)

```cpp
// 在 robot 节点上右键:
menu.addAction(tr("移动"), this, &MainWindow::OnMoveRobot);

// 在 workpiece 节点上右键:
menu.addAction(tr("移动"), this, &MainWindow::OnMoveWorkpiece);
```

### OnMoveRobot / OnMoveWorkpiece

```cpp
void MainWindow::OnMoveRobot() {
    move_target_ = MoveTarget::kRobot;
    gp_Trsf current = controller_->GetBaseTrsf();

    movement_panel_ = new MovementPanel(tr("IRB140"), current, this);
    addDockWidget(Qt::RightDockWidgetArea, movement_panel_);

    connect(movement_panel_, &MovementPanel::PreviewRequested,
            this, &MainWindow::OnMovePreview);
    connect(movement_panel_, &MovementPanel::Accepted,
            this, &MainWindow::OnMoveAccepted);
    connect(movement_panel_, &MovementPanel::Cancelled,
            this, &MainWindow::OnMoveCancelled);
}
```

### OnMovePreview (预览，不记 undo)

```cpp
void MainWindow::OnMovePreview(const gp_Trsf& trsf) {
    if (move_target_ == MoveTarget::kRobot) {
        controller_->SetBaseTrsf(trsf);  // 直接设置，不走 undo
    } else if (move_target_ == MoveTarget::kWorkpiece) {
        SetWorkpieceTrsf(trsf);  // 直接设置，不走 undo
    }
}
```

### OnMoveAccepted (确定，记 undo)

```cpp
void MainWindow::OnMoveAccepted(const gp_Trsf& trsf) {
    gp_Trsf old_trsf = movement_panel_->GetInitialTrsf();

    if (move_target_ == MoveTarget::kRobot) {
        undo_stack_->push(new CmdMoveBase(controller_, old_trsf, trsf));
        // Base 变了 → 重新规划 trajectory (如果有)
        if (!trajectory_.points.empty())
            ReplanTrajectory();
    } else if (move_target_ == MoveTarget::kWorkpiece) {
        auto apply = [this](const gp_Trsf& t) { SetWorkpieceTrsf(t); };
        undo_stack_->push(new CmdMoveWorkpiece(apply, old_trsf, trsf));
        // Workpiece 变了 → 变换 waypoints + trajectory
        TransformWaypointsAndTrajectory(old_trsf, trsf);
    }

    // 销毁面板
    movement_panel_->deleteLater();
    movement_panel_ = nullptr;
    move_target_ = MoveTarget::kNone;
}
```

### OnMoveCancelled (取消，恢复)

```cpp
void MainWindow::OnMoveCancelled() {
    gp_Trsf old_trsf = movement_panel_->GetInitialTrsf();

    if (move_target_ == MoveTarget::kRobot)
        controller_->SetBaseTrsf(old_trsf);
    else if (move_target_ == MoveTarget::kWorkpiece)
        SetWorkpieceTrsf(old_trsf);

    movement_panel_->deleteLater();
    movement_panel_ = nullptr;
    move_target_ = MoveTarget::kNone;
}
```

---

## Coordinate Transform Details

### IK target (世界 → 机器人 base)

```cpp
// OnPoseEdited() 和 TrajectoryPlanner 调用 IK 前:
gp_Trsf target_base = controller_->GetBaseTrsf().Inverted();
target_base.Multiply(target_world);
```

### TCP Pose 显示 (base → 世界)

```cpp
gp_Trsf tcp_world = controller_->GetBaseTrsf();
tcp_world.Multiply(fk.back());
if (tcp_ref_mode_ == 1)
    tcp_world.Multiply(controller_->GetToolTcpTrsf());
```

### Waypoint 生成 (工件局部 → 世界)

```cpp
// OnGenerateWaypoints() 中, Generate() 返回工件局部坐标
for (auto& wp : waypoints_) {
    gp_Trsf world_pose = workpiece_trsf_;
    world_pose.Multiply(wp.pose);
    wp.pose = world_pose;
}
```

### 工件移动后 waypoints 跟随变换

```cpp
void MainWindow::TransformWaypointsAndTrajectory(
    const gp_Trsf& old_trsf, const gp_Trsf& new_trsf)
{
    // delta = new * old^-1
    gp_Trsf delta = new_trsf;
    delta.Multiply(old_trsf.Inverted());

    for (auto& wp : waypoints_) {
        gp_Trsf t = delta;
        t.Multiply(wp.pose);
        wp.pose = t;
    }
    DisplayWaypoints();  // 刷新显示

    // Trajectory points 同样变换 tcp_pose, 重新求 IK
    if (!trajectory_.points.empty())
        ReplanTrajectory();
}
```

### 机器人 base 移动后 trajectory 自动重新规划

```cpp
void MainWindow::ReplanTrajectory() {
    if (waypoints_.empty()) return;
    OnPlanTrajectory(planner_config_.approach_dist);
    // OnPlanTrajectory 内部已处理 base_inv * waypoint 变换
}
```

### 轨迹规划中的坐标变换

```cpp
// OnPlanTrajectory() 中:
gp_Trsf base_inv = controller_->GetBaseTrsf().Inverted();
std::vector<nl::occ::Waypoint> base_wps = waypoints_;
for (auto& wp : base_wps) {
    gp_Trsf t = base_inv;
    t.Multiply(wp.pose);
    wp.pose = t;
}
// 传 base_wps 给 TrajectoryPlanner::Plan()
```

---

## SetWorkpieceTrsf Implementation

```cpp
void MainWindow::SetWorkpieceTrsf(const gp_Trsf& trsf)
{
    workpiece_trsf_ = trsf;
    auto ctx = viewer_->GetContext();
    if (!workpiece_ais_.IsNull())
        ctx->SetLocation(workpiece_ais_, TopLoc_Location(trsf));
    if (!selected_face_ais_.IsNull())
        ctx->SetLocation(selected_face_ais_, TopLoc_Location(trsf));
    ctx->UpdateCurrentViewer();
}
```

---

## Implementation Order

1. **Step 1: MovementPanel** — 新建 `src/ui/MovementPanel.h/.cpp` (UI + signals)
2. **Step 2: RobotController** — `base_trsf_` + `SetBaseTrsf()` + 修改 UpdateRobotDisplay/UpdateCoordinateFrames
3. **Step 3: Commands** — CmdMoveBase + CmdMoveWorkpiece in `src/ui/Commands.h/.cpp`
4. **Step 4: MainWindow** — workpiece_trsf_ + 右键菜单 + 面板创建/销毁 + 信号接线
5. **Step 5: 坐标变换集成** — IK/TCP/Waypoint/Trajectory 的坐标系转换
6. **Step 6: CMakeLists.txt** — 添加 MovementPanel.h/.cpp
7. **Step 7: Build + Test**
8. **Step 8: architecture.md** 更新

---

## Verification

### Build & Test
```bash
cmake --build build --config Release
cd build && ctest --output-on-failure -C Release
```

### Manual Testing

| Step | Expected |
|------|----------|
| 1. Load robot + tool | 显示在原点 |
| 2. 右键 robot → 移动 | 面板弹出, 初始值 0,0,0,0,0,0 |
| 3. X=500, 失焦 | 机器人实时预览移到 X=500 |
| 4. 点确定 | 面板关闭, undo 栈记录一条 |
| 5. Ctrl+Z | 机器人回原点 |
| 6. Import workpiece | 显示在原点 |
| 7. 右键 workpiece → 移动, X=300 | 工件预览移到 X=300 |
| 8. 点取消 | 工件回原点, 面板关闭 |
| 9. 再次移动 workpiece X=300, 确定 | 工件移到 X=300 |
| 10. Select Face → Generate Waypoints | waypoints 在工件新位置正确生成 |
| 11. Plan Trajectory → Play | 机器人正确执行轨迹 |
| 12. 右键 robot → 移动 X=500, 确定 | 机器人移动, trajectory 自动重新规划 |
| 13. 右键 workpiece → 移动 Y=200, 确定 | 工件移动, waypoints 跟着变换, trajectory 重新规划 |
| 14. Ctrl+Z 多次 | 逐步恢复所有操作 |
