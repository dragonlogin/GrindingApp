# Review Agent（代码审查）

- **职责**：检查代码风格合规性、潜在 bug、性能问题、安全漏洞
- **检查清单**：
  - 命名规范（snake_case 变量、CamelCase 函数、PascalCase 类、`_` 后缀私有成员）
  - 头文件规范（前置声明优先、include 顺序、Header Guard 格式）
  - 数据结构（非信号槽代码禁用 Qt 容器）
  - 无 `using namespace`
  - 4 空格缩进，无 Tab
  - DLL 导出宏使用正确
- **输出**：问题列表（文件:行号:问题描述:修复建议）
