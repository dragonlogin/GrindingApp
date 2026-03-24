# 项目开发规范

## 1. 代码风格 (Google C++ Style)

### 命名规范
- 变量名使用 `snake_case`（如 `frame_count`）
- 函数名使用 `CamelCase`（如 `GetFrameCount`）
- 类名使用 `PascalCase`
- 私有成员变量加下划线后缀（如 `member_variable_`）

### 格式化
- 缩进使用 4 个空格，禁止使用 Tab
- 禁止使用 `using namespace`
- namespace 内的代码不要缩进

### 头文件规范
- 使用 `#define` 保护（Header Guards），格式为 `PROJECT_PATH_FILE_H_`
- 非必要尽量不要在头文件中包含头文件
- **第一优先：前置声明** — 只用到指针或引用时，禁止 `#include`，改用前置声明：
  ```cpp
  // ✅
  class Bar;
  class Foo { Bar* bar_; };

  // ❌ 禁止
  #include "bar.h"
  class Foo { Bar* bar_; };
  ```
- **第二优先：struct Rep（Pimpl）** — 必须按值持有第三方类型时，用 `struct Rep` 把实现隔离到 CPP：
  ```cpp
  // foo.h ✅
  struct Rep;
  class Foo { Rep* rep_; };

  // foo.cpp
  #include "third_party.h"
  struct Rep { ThirdParty obj; };
  ```
- **禁止**直接在头文件 `#include` 第三方库头文件，除非该类型出现在公开接口的参数或返回值中
- 头文件的包含顺序：当前 CPP 头文件 → 三方库头文件 → 非本文件夹头文件 → 本文件头文件

---

## 2. 数据结构要求 (Raw Structures)

### 数据结构（强制）

非 Qt 信号槽代码中，**禁止**使用以下类型：
- `QList` → 改用 `std::vector`
- `QVector` → 改用 `std::vector`  
- `QString` → 改用 `std::string`
- `QVariant` → 改用具体类型或 `std::any`
- `QMap` → 改用 `std::map` / `std::unordered_map`

允许例外：`connect()` 的信号槽参数、`QObject` 子类的 `Q_PROPERTY`

生成代码前先判断：这个函数是否直接服务于信号槽？否则一律用 std 类型。

**禁止过度封装**：避免不必要的复杂对象包装，保持内存布局透明。

---

## 3. 单元测试要求 (Unit Testing)

- **强制测试**：每次修改类实现后，必须同步更新或创建对应的单元测试文件
- **框架**：使用 Qt Test 框架（QTest）
- **CMake 集成**：自动在 `CMakeLists.txt` 中添加 `add_test` 和 `target_link_libraries(xxx Qt6::Test)`

---

## 4. 自动工作流

修改代码后，必须自主执行以下命令序列：

1. `cmake --build ${BUILD_DIR} --config Release`（编译，`${BUILD_DIR}` 为实际构建目录，请按项目修改，默认为 `build`）
2. `ctest --output-on-failure`（测试）

如果测试失败，直接读取错误日志并原地修复代码，不要停下来询问。
