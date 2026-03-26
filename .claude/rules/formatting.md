# 格式化规范

- 缩进使用 4 个空格，禁止使用 Tab
- 禁止使用 `using namespace`（用 `using` 声明代替，如 `using nl::core::RbRobot;`）
- namespace 内的代码不要缩进：
  ```cpp
  namespace nl {
  namespace core {

  class Foo {};  // 不缩进

  } // namespace core
  } // namespace nl
  ```
- 左花括号不换行（函数定义除外）
- 函数定义左花括号换行：
  ```cpp
  void Foo()
  {
      ...
  }
  ```
