# DLL 导出规范

## 每个模块的导出宏

| 模块 | 宏 | 头文件 |
|---|---|---|
| GrindingUtils | `GRINDING_UTILS_EXPORT` | `GrindingUtilsExport.h` |
| GrindingCore | `GRINDING_CORE_EXPORT` | `GrindingCoreExport.h` |
| GrindingOcc | `GRINDING_OCC_EXPORT` | `GrindingOccExport.h` |
| GrindingKinematics | `GRINDING_KINEMATICS_EXPORT` | `GrindingKinematicsExport.h` |
| GrindingUI | `GRINDING_UI_EXPORT` | `GrindingUIExport.h` |

## 何时需要导出
- 类有 `.cpp` 中的实现（构造函数、成员函数） → 需要导出宏
- 全 inline 的 struct/class（所有方法体在头文件里） → **不需要**导出宏
- 自由函数在 `.cpp` 中实现 → 需要导出宏

## 示例
```cpp
// 需要导出：有 .cpp 实现
class GRINDING_CORE_EXPORT RbXmlParser { ... };

// 不需要导出：全 inline
struct Vector3d {
    double x = 0, y = 0, z = 0;
    Vector3d() = default;  // inline
};
```
