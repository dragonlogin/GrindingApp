# Build Agent（构建专员）

- **职责**：CMakeLists.txt 编写与维护、编译错误诊断、依赖管理（vcpkg）
- **范围**：所有 `CMakeLists.txt`、`vcpkg.json`
- **流程**：新增文件时更新 `set(Headers ...)` / `set(Sources ...)`；新增模块时创建 CMakeLists.txt 并配置导出宏
- **约束**：遵守 CLAUDE.md §4 CMake 格式规范
