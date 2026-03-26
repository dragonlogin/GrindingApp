# GrindingApp 开发规范

## 构建命令

```bash
"D:/Program Files/Microsoft Visual Studio/2022/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" --build build --config Release
cd build && "D:/Program Files/Microsoft Visual Studio/2022/Professional/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/ctest.exe" --output-on-failure -C Release
```

修改代码后必须自主执行编译+测试。测试失败直接修复，不要停下来询问。

## 规范索引

详细规范按需读取，在 `.claude/rules/` 目录下：

| 文件 | 何时读取 |
|---|---|
| `naming.md` | 写任何代码时 |
| `formatting.md` | 写任何代码时 |
| `namespaces.md` | 新增类、跨模块引用时 |
| `headers.md` | 新增/修改 `.h` 文件时 |
| `data-types.md` | 选择容器/字符串类型时 |
| `dll-export.md` | 新增导出类/函数时 |
| `signal-slot.md` | 写 Qt 信号槽代码时 |
| `cmake-format.md` | 修改 CMakeLists.txt 时 |
| `testing.md` | 写/改测试时 |
| `doc-sync.md` | 新增/删除文件、改目录结构时 |

## Agent 角色索引

复杂任务时按需读取 `.claude/agents/` 下的角色定义：

| 文件 | 角色 |
|---|---|
| `architect.md` | 架构师 — 需求分析、方案设计 |
| `ui.md` | UI — Qt 布局、交互 |
| `algorithm.md` | 算法 — FK/IK、路径规划 |
| `occ.md` | 3D — OCCT 几何、场景 |
| `core.md` | 数据 — 模型解析、工具类 |
| `test.md` | 测试 — 编写运行测试 |
| `build.md` | 构建 — CMake、编译 |
| `review.md` | 审查 — 代码质量检查 |
| `doc.md` | 文档 — architecture.md |
| `workflow.md` | 协作流程图 |
