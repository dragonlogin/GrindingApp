# 单元测试要求

## 强制规则
- 每次修改类实现后，必须同步更新或创建对应的单元测试文件
- 测试文件名：`Test` + 类名 + `.cpp`（如 `TestRobotKinematics.cpp`）
- 测试放在 `tests/` 目录，单独写 `CMakeLists.txt`

## 框架
- 使用 Qt Test 框架（QTest）
- `QTEST_MAIN(TestClassName)` + `#include "TestClassName.moc"`

## CMake 集成
```cmake
add_executable(TestFoo TestFoo.cpp)
target_link_libraries(TestFoo Qt5::Test GrindingXxx)
add_test(NAME TestFoo COMMAND TestFoo)
```

## 用例设计
- 正常值：常规输入，验证期望输出
- 边界值：极限输入（0、空、最大值）
- 退化情况：奇异点、无解、溢出
- Round-trip：FK→IK→FK 或 encode→decode→encode
