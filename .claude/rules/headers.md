# 头文件规范

## Header Guard
- 格式：`PROJECT_PATH_FILE_H_`
- 例：`GRINDINGAPP_SRC_UI_JOG_PANEL_H_`

## Include 最小化（按优先级）

1. **前置声明**（只用到指针/引用时，禁止 include）：
   ```cpp
   // OK
   class Bar;
   class Foo { Bar* bar_; };

   // 禁止
   #include "bar.h"
   class Foo { Bar* bar_; };
   ```

2. **struct Rep (Pimpl)**（必须按值持有第三方类型时）：
   ```cpp
   // foo.h
   struct Rep;
   class Foo { Rep* rep_; };

   // foo.cpp
   #include "third_party.h"
   struct Rep { ThirdParty obj; };
   ```

3. **禁止**直接在头文件 `#include` 第三方库头文件，除非该类型出现在公开接口的参数或返回值中

## Include 顺序
1. 当前 CPP 对应的头文件
2. 三方库头文件（Eigen、OCCT、Qt 等）
3. 非本文件夹的项目头文件
4. 本文件夹的项目头文件
