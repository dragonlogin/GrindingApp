# Waypoint Generation with Bridge Pattern — Design Document

## Context

Current `SurfaceWaypointGen` provides a single hardcoded strategy: `GenerateGridWaypoints()` with auto-selection of the largest face. User needs:

1. **Manual surface selection** via 3D click picking
2. **Multiple generation algorithms**: Grid + Planar Cut
3. **Extensible architecture** for future dimensions (input types, output formats, parameters)

This design adopts a **simplified Bridge pattern** to separate the generation abstraction from algorithm implementations.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│  Abstraction                                             │
│  ┌─────────────────────────────────────────────────┐    │
│  │  WaypointGenerator                               │    │
│  │  - SetFace(TopoDS_Face)                          │    │
│  │  - SetAlgorithm(IWaypointAlgo*)                  │    │
│  │  - Generate(WaypointConfig) → vector<Waypoint>   │    │
│  └─────────────────────────────────────────────────┘    │
│                          │                               │
│                          ▼                               │
│  Implementation (Algorithm)                              │
│  ┌─────────────────┐    ┌─────────────────┐            │
│  │ IWaypointAlgo   │    │ IWaypointAlgo   │            │
│  │ ─────────────── │    │ ─────────────── │            │
│  │ GridAlgo        │    │ PlanarCutAlgo   │            │
│  │ - u_steps       │    │ - cut_normal    │            │
│  │ - v_steps       │    │ - num_slices    │            │
│  └─────────────────┘    └─────────────────┘            │
└─────────────────────────────────────────────────────────┘
```

---

## File Structure

```
src/occ/
├── Waypoint.h                    # Data structure (existing)
├── IWaypointAlgo.h               # Algorithm interface (new)
├── WaypointGridAlgo.h/.cpp       # Grid algorithm (new)
├── WaypointPlanarAlgo.h/.cpp     # Planar cut algorithm (new)
├── WaypointGenerator.h/.cpp      # Bridge upper class (new)
└── SurfaceWaypointGen.h/.cpp     # Keep LargestFace utility (existing)
```

---

## Core Interfaces

### IWaypointAlgo.h

```cpp
#ifndef GRINDINGAPP_SRC_OCC_IWAYPOINT_ALGO_H_
#define GRINDINGAPP_SRC_OCC_IWAYPOINT_ALGO_H_

#include <vector>
#include <gp_Dir.hxx>
#include <TopoDS_Face.hxx>
#include "Waypoint.h"

namespace nl {
namespace occ {

struct WaypointConfig {
    int u_steps = 10;           // Grid: U direction steps
    int v_steps = 5;            // Grid: V direction steps
    int num_slices = 10;        // Planar: number of cut planes
    gp_Dir cut_normal{0, 0, 1}; // Planar: cut direction
    double approach_dist = 0.0; // Offset along normal (mm)
};

class IWaypointAlgo {
public:
    virtual ~IWaypointAlgo() = default;
    virtual std::vector<Waypoint> Generate(
        const TopoDS_Face& face,
        const WaypointConfig& config) = 0;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_IWAYPOINT_ALGO_H_
```

### WaypointGenerator.h

```cpp
#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_GENERATOR_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_GENERATOR_H_

#include <memory>
#include "IWaypointAlgo.h"
#include "GrindingOccExport.h"

namespace nl {
namespace occ {

class GRINDING_OCC_EXPORT WaypointGenerator {
public:
    void SetFace(const TopoDS_Face& face);
    void SetAlgorithm(std::unique_ptr<IWaypointAlgo> algo);

    std::vector<Waypoint> Generate(const WaypointConfig& config);

private:
    TopoDS_Face face_;
    std::unique_ptr<IWaypointAlgo> algo_;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_WAYPOINT_GENERATOR_H_
```

### WaypointGridAlgo.h

```cpp
#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_GRID_ALGO_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_GRID_ALGO_H_

#include "IWaypointAlgo.h"

namespace nl {
namespace occ {

class WaypointGridAlgo : public IWaypointAlgo {
public:
    std::vector<Waypoint> Generate(
        const TopoDS_Face& face,
        const WaypointConfig& config) override;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_WAYPOINT_GRID_ALGO_H_
```

### WaypointPlanarAlgo.h

```cpp
#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_PLANAR_ALGO_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_PLANAR_ALGO_H_

#include "IWaypointAlgo.h"

namespace nl {
namespace occ {

class WaypointPlanarAlgo : public IWaypointAlgo {
public:
    std::vector<Waypoint> Generate(
        const TopoDS_Face& face,
        const WaypointConfig& config) override;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_WAYPOINT_PLANAR_ALGO_H_
```

---

## Algorithm Implementations

### WaypointGridAlgo

Samples surface parametric space uniformly in U and V directions:

1. Use `BRepAdaptor_Surface` to get U/V parameter range
2. Sample points on uniform grid
3. Use `BRepLProp_SLProps` to compute position and normal at each point
4. Handle face orientation (reverse normal if `TopAbs_REVERSED`)
5. Build TCP frame: Z = surface normal, X = U-tangent
6. Apply approach distance offset along normal
7. Return waypoints in serpentine pattern (alternating row direction)

### WaypointPlanarAlgo

Cuts surface with parallel planes, generates waypoints along intersection curves:

1. Compute bounding box, project corners onto cut direction to get range
2. For each cut plane at regular intervals:
   - Use `BRepAlgoAPI_Section` to intersect plane with face
   - Traverse resulting edges
   - Sample points along each edge using `BRepAdaptor_Curve`
   - Build TCP frame: Z = cut normal (simplified)
3. Return all sampled waypoints

---

## 3D Surface Picking

### Flow

```
User clicks 3D view
       │
       ▼
OcctViewWidget::mousePressEvent()
       │
       ▼
AIS_InteractiveContext::Select()
       │
       ▼
emit ShapeSelected()
       │
       ▼
MainWindow::OnFacePicked()
       │
       ▼
Handle(StdSelect_BRepOwner) → TopoDS_Face
       │
       ▼
Store as selected_face_, highlight in yellow
```

### OcctViewWidget Signal

```cpp
signals:
    void ShapeSelected();
```

### MainWindow Handling

```cpp
void MainWindow::OnFacePicked() {
    if (selection_mode_ != SelectionMode::kSelectFace) return;

    auto ctx = viewer_->GetContext();
    ctx->Select();

    Handle(StdSelect_BRepOwner) owner =
        Handle(StdSelect_BRepOwner)::DownCast(ctx->SelectedOwner());

    if (owner.IsNull() || owner->Shape().ShapeType() != TopAbs_FACE) {
        statusBar()->showMessage(tr("Please select a valid surface"));
        return;
    }

    selected_face_ = TopoDS::Face(owner->Shape());

    // Highlight
    if (!selected_face_ais_.IsNull()) {
        ctx->Remove(selected_face_ais_, false);
    }
    selected_face_ais_ = new AIS_Shape(selected_face_);
    ctx->Display(selected_face_ais_, AIS_Shaded, 0, false);
    ctx->SetColor(selected_face_ais_, Quantity_NOC_YELLOW, false);

    selection_mode_ = SelectionMode::kNone;
    statusBar()->showMessage(tr("Surface selected, ready to generate waypoints"));
}
```

---

## MainWindow Integration

### New Members

```cpp
// MainWindow.h
class MainWindow : public QMainWindow {
    Q_OBJECT

private:
    // === Surface Picking ===
    enum class SelectionMode { kNone, kSelectFace };
    SelectionMode selection_mode_ = SelectionMode::kNone;
    TopoDS_Face selected_face_;
    Handle(AIS_Shape) selected_face_ais_;

    // === Waypoint Generation ===
    enum class WaypointMode { kGrid, kPlanarCut };
    WaypointMode waypoint_mode_ = WaypointMode::kGrid;
    std::unique_ptr<nl::occ::WaypointGenerator> waypoint_gen_;
    nl::occ::WaypointConfig waypoint_config_;

    // === Methods ===
    void SetupWaypointMenu();
    void OnSelectFaceMode();
    void OnFacePicked();
    void OnGenerateWaypoints();
    void OnSetGridMode();
    void OnSetPlanarMode();
    void OnClearWaypoints();
    void DisplayWaypoints();
};
```

### Menu Structure

```
Path
├── Select Face             → OnSelectFaceMode()
├── ───────────────
├── Generation Mode
│   ├── Grid (checked)      → OnSetGridMode()
│   └── Planar Cut          → OnSetPlanarMode()
├── ───────────────
├── Generate Waypoints      → OnGenerateWaypoints()
└── Clear Waypoints         → OnClearWaypoints()
```

### OnGenerateWaypoints Implementation

```cpp
void MainWindow::OnGenerateWaypoints() {
    if (selected_face_.IsNull()) {
        QMessageBox::warning(this, tr("Warning"), tr("Please select a surface first"));
        return;
    }

    waypoint_gen_ = std::make_unique<nl::occ::WaypointGenerator>();
    waypoint_gen_->SetFace(selected_face_);

    if (waypoint_mode_ == WaypointMode::kGrid) {
        waypoint_gen_->SetAlgorithm(std::make_unique<nl::occ::WaypointGridAlgo>());
    } else {
        waypoint_gen_->SetAlgorithm(std::make_unique<nl::occ::WaypointPlanarAlgo>());
    }

    waypoints_ = waypoint_gen_->Generate(waypoint_config_);

    if (waypoints_.empty()) {
        statusBar()->showMessage(tr("No waypoints generated for this surface"));
        return;
    }

    DisplayWaypoints();
    statusBar()->showMessage(tr("%1 waypoints generated").arg(waypoints_.size()));
}
```

---

## Error Handling

### Surface Picking Errors

| Case | Handling |
|------|----------|
| No workpiece loaded | Disable menu or show `tr("No workpiece loaded")` |
| Click on empty space | Status bar: `tr("Please click on a valid surface")` |
| Selected shape is not Face | Filter `TopAbs_FACE`, ignore others |

### Waypoint Generation Errors

| Case | Handling |
|------|----------|
| No surface selected | Show warning dialog |
| Normal undefined at point | `BRepLProp_SLProps::IsNormalDefined()` check, skip singular points |
| No intersection for cut plane | `BRepAlgoAPI_Section::IsDone()` check, skip that layer |
| Degenerate face (zero area) | Pre-check `GProp_GProps::Mass()` > epsilon |
| Empty result | Status bar message, no visualization |

---

## CMakeLists.txt Changes

### src/occ/CMakeLists.txt

```cmake
set(HEADERS
    GrindingOccExport.h
    StepImporter.h
    OcctViewWidget.h
    Waypoint.h
    IWaypointAlgo.h
    WaypointGenerator.h
    WaypointGridAlgo.h
    WaypointPlanarAlgo.h
    SurfaceWaypointGen.h
)

set(SOURCES
    StepImporter.cpp
    OcctViewWidget.cpp
    WaypointGenerator.cpp
    WaypointGridAlgo.cpp
    WaypointPlanarAlgo.cpp
    SurfaceWaypointGen.cpp
)
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
| 1. Launch app | Main window displays normally |
| 2. Import STEP workpiece | Workpiece visible in 3D view |
| 3. Click Path → Select Face | Status bar prompts to select surface |
| 4. Click workpiece surface | Surface highlighted yellow, status confirms selection |
| 5. Path → Generate Waypoints (default Grid) | Yellow polyline path displayed |
| 6. Switch to Planar Cut, regenerate | Path displayed with planar cut pattern |
| 7. Path → Clear Waypoints | Path removed from view |

### Unit Tests (Optional)

```cpp
// tests/TestWaypointGenerator.cpp

void TestGridAlgo::testGenerateOnPlane() {
    gp_Pln pln(gp::Origin(), gp::DZ());
    TopoDS_Face face = BRepBuilderAPI_MakeFace(pln, -10, 10, -10, 10);

    nl::occ::WaypointConfig config;
    config.u_steps = 5;
    config.v_steps = 3;

    nl::occ::WaypointGridAlgo algo;
    auto waypoints = algo.Generate(face, config);

    QCOMPARE(waypoints.size(), (size_t)24);  // (5+1) * (3+1)
}

void TestPlanarAlgo::testGenerateOnCylinder() {
    gp_Cylinder cyl(gp::XOY(), 5.0);
    TopoDS_Face face = BRepBuilderAPI_MakeFace(cyl, 0, 2*M_PI, 0, 10);

    nl::occ::WaypointConfig config;
    config.num_slices = 5;
    config.cut_normal = gp_Dir(0, 0, 1);

    nl::occ::WaypointPlanarAlgo algo;
    auto waypoints = algo.Generate(face, config);

    QVERIFY(!waypoints.empty());
}
```

---

## Future Extensions

The Bridge pattern enables independent extension along multiple dimensions:

| Dimension | Future Extensions |
|-----------|-------------------|
| Algorithm | Spiral, Contour, Adaptive density |
| Input | Whole shape, Wire, Point cloud |
| Output | Joint angles, CSV file, ROS trajectory |
| Parameters | JSON config, UI dialog |

Each extension only requires adding a new implementation class without modifying existing code.

---

## File Checklist

| Action | File |
|--------|------|
| New | `src/occ/IWaypointAlgo.h` |
| New | `src/occ/WaypointGenerator.h` |
| New | `src/occ/WaypointGenerator.cpp` |
| New | `src/occ/WaypointGridAlgo.h` |
| New | `src/occ/WaypointGridAlgo.cpp` |
| New | `src/occ/WaypointPlanarAlgo.h` |
| New | `src/occ/WaypointPlanarAlgo.cpp` |
| Modify | `src/occ/CMakeLists.txt` |
| Modify | `src/ui/MainWindow.h` |
| Modify | `src/ui/MainWindow.cpp` |
| Modify | `src/occ/OcctViewWidget.h` (add signal) |
| Modify | `src/occ/OcctViewWidget.cpp` (emit signal) |
| Optional | `tests/TestWaypointGenerator.cpp` |
| Update | `architecture.md` |
