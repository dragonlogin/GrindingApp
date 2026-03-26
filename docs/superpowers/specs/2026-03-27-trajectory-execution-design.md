# Trajectory Execution & Playback — Design Document

## Context

Waypoint generation (Bridge pattern) is complete. Generated waypoints are displayed as a static yellow polyline, but the robot cannot yet execute the path. User needs:

1. **IK solving** for each waypoint, with anomaly detection and manual solution editing
2. **Mixed motion**: MoveJ for approach, MoveL for grinding path
3. **Real-time playback** with transport controls (play/pause/stop/scrub)

---

## Architecture Overview

```
waypoints_ (from WaypointGenerator)
       │
       ▼
TrajectoryPlanner::Plan()
       │
       ├── 1. Approach point = wp[0].pose offset by -approach_dist along Z
       ├── 2. MoveJ: current_Q → IK(approach) → joint-space interpolation
       ├── 3. MoveL: wp[i] → wp[i+1] → Cartesian interpolation → IK per step
       ├── 4. Auto-select closest IK solution (seed = previous point)
       └── 5. Flag anomalies: IK failure / joint jump > threshold
       │
       ▼
Trajectory { vector<TrajectoryPoint> }
       │
       ├──► TrajectoryPanel (right Dock)
       │    ├── Table view: pose + joints + status
       │    ├── Red highlight on anomaly rows
       │    └── Click row → preview pose + switch IK solution
       │
       └──► TrajectoryPlayer (bottom Dock)
            ├── QTimer-driven frame advance
            ├── Play / Pause / Stop + progress slider + speed control
            └── Each tick → RobotController::SetJointAngles(points[i].joint_angles)
```

---

## File Structure

```
src/occ/
├── Trajectory.h                  # Data structures (new, header-only)
├── TrajectoryPlanner.h/.cpp      # MoveJ/MoveL + IK + anomaly detection (new)

src/ui/
├── TrajectoryPanel.h/.cpp        # Right dock: table + IK editing (new)
├── TrajectoryPlayer.h/.cpp       # Bottom dock: playback controls (new)
├── MainWindow.h/.cpp             # Integration (modify)
```

---

## Core Data Structures

### Trajectory.h

```cpp
#ifndef GRINDINGAPP_SRC_OCC_TRAJECTORY_H_
#define GRINDINGAPP_SRC_OCC_TRAJECTORY_H_

#include <vector>
#include <gp_Trsf.hxx>
#include "Q.h"

namespace nl {
namespace occ {

struct TrajectoryPoint {
    gp_Trsf tcp_pose;
    nl::utils::Q joint_angles;

    enum class MoveType { kMoveJ, kMoveL };
    MoveType move_type = MoveType::kMoveL;

    enum class Status { kOk, kIkFailed, kJointJump };
    Status status = Status::kOk;

    int waypoint_index = -1;  // Source waypoint index (-1 = interpolated)
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;

    bool HasErrors() const {
        for (const auto& p : points)
            if (p.status != TrajectoryPoint::Status::kOk) return true;
        return false;
    }

    int ErrorCount() const {
        int count = 0;
        for (const auto& p : points)
            if (p.status != TrajectoryPoint::Status::kOk) ++count;
        return count;
    }
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_TRAJECTORY_H_
```

---

## Core Interfaces

### TrajectoryPlanner.h

```cpp
#ifndef GRINDINGAPP_SRC_OCC_TRAJECTORY_PLANNER_H_
#define GRINDINGAPP_SRC_OCC_TRAJECTORY_PLANNER_H_

#include "Trajectory.h"
#include "Waypoint.h"
#include "RbXmlParser.h"
#include "GrindingOccExport.h"

namespace nl {
namespace occ {

class GRINDING_OCC_EXPORT TrajectoryPlanner {
public:
    struct Config {
        double approach_dist = 50.0;        // mm, along first wp normal
        int movej_steps = 50;               // interpolation steps for MoveJ
        int movel_steps_per_seg = 10;       // steps per MoveL segment
        double joint_jump_threshold = 30.0; // degrees, flag if exceeded
    };

    Trajectory Plan(
        const std::vector<Waypoint>& waypoints,
        const nl::core::RbRobot& robot,
        const nl::utils::Q& current_angles,
        const Config& config);

    // Re-solve one point with a specific IK solution index
    bool ResolveSinglePoint(
        Trajectory& traj, int index,
        const nl::core::RbRobot& robot,
        int solution_index);

private:
    // MoveJ: joint-space linear interpolation
    std::vector<TrajectoryPoint> InterpolateMoveJ(
        const nl::utils::Q& from,
        const nl::utils::Q& to,
        const gp_Trsf& target_pose,
        int steps);

    // MoveL: Cartesian SLERP + linear position interpolation
    std::vector<TrajectoryPoint> InterpolateMoveL(
        const gp_Trsf& from_pose,
        const gp_Trsf& to_pose,
        const nl::core::RbRobot& robot,
        const nl::utils::Q& seed_angles,
        int steps);

    // Compute approach point: offset along -Z of first waypoint
    gp_Trsf ComputeApproachPose(
        const gp_Trsf& first_wp_pose,
        double approach_dist);

    // Check if joint jump between two Q exceeds threshold
    bool HasJointJump(
        const nl::utils::Q& a,
        const nl::utils::Q& b,
        double threshold);
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_TRAJECTORY_PLANNER_H_
```

---

## Algorithm Implementations

### TrajectoryPlanner::Plan()

1. **Compute approach point**:
   - `approach_pose = first_waypoint.pose`
   - Translate `-approach_dist` along the pose's Z axis (retract along normal)

2. **IK for approach point**:
   - `ComputeIk(robot, approach_pose, current_angles, approach_Q)`
   - If IK fails, mark first point as `kIkFailed`

3. **MoveJ segment** (current → approach):
   - Linear interpolation in joint space: `Q_i = (1-t)*current_Q + t*approach_Q`
   - `movej_steps` intermediate points
   - All marked `MoveType::kMoveJ`

4. **MoveL segments** (waypoint to waypoint):
   - For each pair `(wp[i], wp[i+1])`:
     - Position: linear interpolation `P(t) = (1-t)*P_i + t*P_{i+1}`
     - Rotation: SLERP via quaternion interpolation
     - IK each intermediate point, seed = previous point's `joint_angles`
     - If IK fails → mark `kIkFailed`
     - If any joint diff > `joint_jump_threshold` → mark `kJointJump`
   - `movel_steps_per_seg` steps per segment

5. **Return** `Trajectory` with all points

### SLERP Implementation

```cpp
// Quaternion SLERP for rotation interpolation
// Extract quaternion from gp_Trsf, interpolate, rebuild gp_Trsf
// Use gp_Quaternion built into OCCT:
gp_Quaternion q1, q2;
q1.SetRotation(from_pose.GetRotation());
q2.SetRotation(to_pose.GetRotation());
gp_Quaternion qi = q1;  // OCCT doesn't have SLERP, implement manually:
// qi = q1 * (q1^-1 * q2)^t
```

### ResolveSinglePoint()

1. Call `ComputeIkAllSolutions(robot, traj.points[index].tcp_pose, seed, solutions)`
2. If `solution_index` valid → update `joint_angles` and re-evaluate status
3. Re-check joint jump with neighbors

---

## TrajectoryPanel (Right Dock)

### Layout

```
┌─ Trajectory Editor ──────────────────────────┐
│                                                │
│  Approach distance: [50.0 ▾] mm   [Plan]      │
│                                                │
│  ┌────┬──────┬────────────────┬─────────┐     │
│  │ #  │ Type │ X Y Z Rx Ry Rz│ Status  │     │
│  ├────┼──────┼────────────────┼─────────┤     │
│  │  0 │ MJ   │ 100 0 200 ... │   OK    │     │
│  │  1 │ MJ   │ 102 1 198 ... │   OK    │     │
│  │ ...│      │                │         │     │
│  │ 50 │ ML   │ 110 5 150 ... │   OK    │◄wp0 │
│  │ 51 │ ML   │ 112 5 148 ... │  JUMP   │◄red │
│  │ ...│      │                │         │     │
│  └────┴──────┴────────────────┴─────────┘     │
│                                                │
│  Selected point IK:                            │
│  Solution: [1/8 ▾]  J1: 30.1  J2: -45.2 ...  │
│                                                │
│  Total: 150 points | Errors: 1                 │
└────────────────────────────────────────────────┘
```

### TrajectoryPanel.h

```cpp
#ifndef GRINDINGAPP_SRC_UI_TRAJECTORY_PANEL_H_
#define GRINDINGAPP_SRC_UI_TRAJECTORY_PANEL_H_

#include <QDockWidget>
#include <QTableWidget>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>

#include "GrindingUIExport.h"
#include "Trajectory.h"

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT TrajectoryPanel : public QDockWidget {
    Q_OBJECT

public:
    explicit TrajectoryPanel(QWidget* parent = nullptr);

    void SetTrajectory(const nl::occ::Trajectory& traj);
    void UpdatePoint(int index, const nl::occ::TrajectoryPoint& point);
    void SetIkSolutions(const std::vector<nl::utils::Q>& solutions);

signals:
    void PlanRequested(double approach_dist);
    void PointSelected(int index);
    void IkSolutionChanged(int point_index, int solution_index);

private:
    void SetupUi();
    void PopulateTable();
    void HighlightRow(int row, nl::occ::TrajectoryPoint::Status status);

    QTableWidget*    table_ = nullptr;
    QDoubleSpinBox*  approach_spin_ = nullptr;
    QPushButton*     plan_btn_ = nullptr;
    QComboBox*       ik_combo_ = nullptr;
    QLabel*          summary_label_ = nullptr;

    nl::occ::Trajectory trajectory_;
    int selected_row_ = -1;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_TRAJECTORY_PANEL_H_
```

---

## TrajectoryPlayer (Bottom Dock)

### Layout

```
┌─ Playback ───────────────────────────────────────────────┐
│  [▶] [⏸] [⏹]  │  ██████████░░░░░░░░ 34/150  │  1.0x ▾  │
│                 │  ← draggable progress bar →  │  speed   │
└──────────────────────────────────────────────────────────┘
```

### TrajectoryPlayer.h

```cpp
#ifndef GRINDINGAPP_SRC_UI_TRAJECTORY_PLAYER_H_
#define GRINDINGAPP_SRC_UI_TRAJECTORY_PLAYER_H_

#include <QDockWidget>
#include <QTimer>
#include <QPushButton>
#include <QSlider>
#include <QLabel>

#include "GrindingUIExport.h"

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT TrajectoryPlayer : public QDockWidget {
    Q_OBJECT

public:
    explicit TrajectoryPlayer(QWidget* parent = nullptr);

    void SetFrameCount(int count);
    void SetCurrentFrame(int frame);

signals:
    void FrameChanged(int index);
    void PlaybackFinished();

private slots:
    void OnPlay();
    void OnPause();
    void OnStop();
    void OnTimerTick();
    void OnSliderMoved(int value);
    void OnSpeedChanged(int value);

private:
    void SetupUi();
    void UpdateLabel();

    QTimer*      timer_ = nullptr;
    QPushButton* play_btn_ = nullptr;
    QPushButton* pause_btn_ = nullptr;
    QPushButton* stop_btn_ = nullptr;
    QSlider*     progress_slider_ = nullptr;
    QSlider*     speed_slider_ = nullptr;
    QLabel*      frame_label_ = nullptr;

    int current_frame_ = 0;
    int frame_count_ = 0;
    double speed_factor_ = 1.0;
    static constexpr int kBaseIntervalMs = 33;  // ~30fps
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_TRAJECTORY_PLAYER_H_
```

---

## MainWindow Integration

### New Members

```cpp
// MainWindow.h
private:
    // === Trajectory ===
    TrajectoryPanel*  traj_panel_ = nullptr;
    TrajectoryPlayer* traj_player_ = nullptr;
    nl::occ::Trajectory trajectory_;
    nl::occ::TrajectoryPlanner::Config planner_config_;

    void SetupTrajectoryPanel();
    void SetupTrajectoryPlayer();
    void OnPlanTrajectory(double approach_dist);
    void OnTrajectoryPointSelected(int index);
    void OnIkSolutionChanged(int point_index, int solution_index);
    void OnPlaybackFrame(int index);
    void OnPlaybackFinished();
    void OnClearTrajectory();
```

### Signal Wiring

```cpp
// In constructor, after SetupWaypointMenu():
SetupTrajectoryPanel();
SetupTrajectoryPlayer();

connect(traj_panel_, &TrajectoryPanel::PlanRequested,
        this, &MainWindow::OnPlanTrajectory);
connect(traj_panel_, &TrajectoryPanel::PointSelected,
        this, &MainWindow::OnTrajectoryPointSelected);
connect(traj_panel_, &TrajectoryPanel::IkSolutionChanged,
        this, &MainWindow::OnIkSolutionChanged);
connect(traj_player_, &TrajectoryPlayer::FrameChanged,
        this, &MainWindow::OnPlaybackFrame);
connect(traj_player_, &TrajectoryPlayer::PlaybackFinished,
        this, &MainWindow::OnPlaybackFinished);
```

### OnPlanTrajectory Implementation

```cpp
void MainWindow::OnPlanTrajectory(double approach_dist) {
    if (waypoints_.empty()) {
        QMessageBox::warning(this, tr("Warning"),
            tr("Generate waypoints first"));
        return;
    }

    planner_config_.approach_dist = approach_dist;

    nl::occ::TrajectoryPlanner planner;
    trajectory_ = planner.Plan(
        waypoints_,
        controller_->GetRobot(),
        controller_->GetJointAngles(),
        planner_config_);

    traj_panel_->SetTrajectory(trajectory_);
    traj_player_->SetFrameCount(
        static_cast<int>(trajectory_.points.size()));

    if (trajectory_.HasErrors()) {
        statusBar()->showMessage(
            tr("Trajectory planned with %1 errors")
                .arg(trajectory_.ErrorCount()), 5000);
    } else {
        statusBar()->showMessage(
            tr("Trajectory planned: %1 points, ready to play")
                .arg(trajectory_.points.size()), 5000);
    }
}
```

### OnPlaybackFrame Implementation

```cpp
void MainWindow::OnPlaybackFrame(int index) {
    if (index < 0 || index >= static_cast<int>(trajectory_.points.size()))
        return;

    const auto& pt = trajectory_.points[index];
    if (pt.status == nl::occ::TrajectoryPoint::Status::kIkFailed)
        return;  // Skip unsolved frames

    controller_->SetJointAngles(pt.joint_angles);
}
```

### OnTrajectoryPointSelected Implementation

```cpp
void MainWindow::OnTrajectoryPointSelected(int index) {
    if (index < 0 || index >= static_cast<int>(trajectory_.points.size()))
        return;

    const auto& pt = trajectory_.points[index];
    if (pt.status != nl::occ::TrajectoryPoint::Status::kIkFailed) {
        controller_->SetJointAngles(pt.joint_angles);
    }

    // Show IK alternatives for this point
    std::vector<nl::utils::Q> solutions;
    nl::kinematics::ComputeIkAllSolutions(
        controller_->GetRobot(), pt.tcp_pose,
        pt.joint_angles, solutions);
    traj_panel_->SetIkSolutions(solutions);
}
```

---

## Menu Structure

### Path Menu (updated)

```
Path
├── Select Face              → OnSelectFaceMode()
├── ─────────────
├── Generation Mode
│   ├── Grid (checked)       → OnSetGridMode()
│   └── Planar Cut           → OnSetPlanarMode()
├── ─────────────
├── Generate Waypoints       → OnGenerateWaypoints()     Ctrl+G
├── Clear Waypoints          → OnClearWaypoints()
├── ─────────────
├── Plan Trajectory          → OnPlanTrajectory()        Ctrl+P
└── Clear Trajectory         → OnClearTrajectory()
```

---

## Error Handling

### Trajectory Planning Errors

| Case | Handling |
|------|----------|
| No waypoints generated | Show warning dialog |
| No robot loaded | Show warning dialog |
| Approach point IK fails | Mark first MoveJ point `kIkFailed`, continue |
| MoveL point IK fails | Mark `kIkFailed`, skip in playback, red in table |
| Joint jump > threshold | Mark `kJointJump`, yellow in table |
| All points failed | Show error, disable playback |

### Playback Errors

| Case | Handling |
|------|----------|
| Play with errors in trajectory | Allow, skip `kIkFailed` frames |
| Drag slider to failed frame | Show status but don't move robot |
| Speed = 0 | Clamp to 0.1x minimum |

---

## CMakeLists.txt Changes

### src/occ/CMakeLists.txt

Add to HEADERS:
```cmake
Trajectory.h
TrajectoryPlanner.h
```

Add to SOURCES:
```cmake
TrajectoryPlanner.cpp
```

### src/ui/CMakeLists.txt

Add to HEADERS:
```cmake
TrajectoryPanel.h
TrajectoryPlayer.h
```

Add to SOURCES:
```cmake
TrajectoryPanel.cpp
TrajectoryPlayer.cpp
```

---

## Verification

### Build & Test

```bash
cmake --build build --config Release
cd build && ctest --output-on-failure -C Release
```

### Manual Testing Steps

| Step | Expected Result |
|------|-----------------|
| 1. Load robot + tool + STEP workpiece | All display in 3D view |
| 2. Select Face → Generate Waypoints (Grid) | Yellow polyline path |
| 3. Path → Plan Trajectory | Table populates, approach + grinding points |
| 4. Check anomaly rows (if any) | Red highlight, click to preview |
| 5. Click anomaly row → switch IK solution | Robot moves, status updates to OK |
| 6. Click Play | Robot animates along trajectory |
| 7. Pause → drag slider → resume | Robot jumps to frame, continues |
| 8. Adjust speed to 2.0x | Animation speeds up |
| 9. Stop | Robot stays at current frame |
| 10. Clear Trajectory | Table clears, player resets |

---

## File Checklist

| Action | File |
|--------|------|
| New | `src/occ/Trajectory.h` |
| New | `src/occ/TrajectoryPlanner.h` |
| New | `src/occ/TrajectoryPlanner.cpp` |
| New | `src/ui/TrajectoryPanel.h` |
| New | `src/ui/TrajectoryPanel.cpp` |
| New | `src/ui/TrajectoryPlayer.h` |
| New | `src/ui/TrajectoryPlayer.cpp` |
| Modify | `src/occ/CMakeLists.txt` |
| Modify | `src/ui/CMakeLists.txt` |
| Modify | `src/ui/MainWindow.h` |
| Modify | `src/ui/MainWindow.cpp` |
| Update | `architecture.md` |

---

## Implementation Order

1. `Trajectory.h` — data structures (header-only)
2. `TrajectoryPlanner.h/.cpp` — MoveJ/MoveL interpolation + IK + anomaly detection
3. `TrajectoryPanel.h/.cpp` — table UI + IK editing
4. `TrajectoryPlayer.h/.cpp` — playback controls + timer
5. MainWindow integration — wire everything together
6. CMakeLists.txt + architecture.md updates
7. Build + test

---

## Implementation Notes (Post-Implementation)

### Architecture Deviation

`TrajectoryPlanner` was moved from `src/occ/` (`nl::occ`) to `src/ui/` (`nl::ui`), because it depends on `GrindingKinematics` (IK solver), while `GrindingOcc` and `GrindingKinematics` are sibling modules that cannot depend on each other. `Trajectory.h` (pure data structures) remains in `src/occ/` as a cross-layer shared type.

### Final File List

| Action | File | Module |
|--------|------|--------|
| New | `src/occ/Trajectory.h` | GrindingOcc (header-only data) |
| New | `src/ui/TrajectoryPlanner.h/.cpp` | GrindingUI (moved from occ due to kinematics dep) |
| New | `src/ui/TrajectoryPanel.h/.cpp` | GrindingUI (right Dock, table + IK editing) |
| New | `src/ui/TrajectoryPlayer.h/.cpp` | GrindingUI (bottom Dock, playback controls) |
| Modify | `src/occ/CMakeLists.txt` | Added Trajectory.h |
| Modify | `src/ui/CMakeLists.txt` | Added TrajectoryPlanner/Panel/Player |
| Modify | `src/ui/MainWindow.h/.cpp` | Integration, signal wiring, menu items |
| Update | `architecture.md` | Directory tree, module tables, task lookup |

### Key Design Points

- **Path menu** extended: Plan Trajectory (`Ctrl+P`) + Clear Trajectory
- **MoveJ segment**: Joint-space linear interpolation (current pose → approach point)
- **MoveL segments**: Quaternion SLERP + linear position interpolation, IK seeded from previous point for continuity
- **Anomaly detection**: `kIkFailed` (IK no solution) and `kJointJump` (any joint > 30° between consecutive points)
- **IK re-solve**: `ResolveSinglePoint()` allows user to switch IK solution for any individual point
- **Playback**: QTimer at ~30fps, speed adjustable 0.1x–3.0x, draggable progress slider
