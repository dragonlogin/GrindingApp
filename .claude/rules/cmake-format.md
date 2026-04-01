# CMakeLists.txt 格式规范

## 源文件声明（强制）

`.h` 和 `.cpp` 分开用 `set()` 声明，再统一传给 `add_library` / `add_executable`：

```cmake
# 正确
set(Headers
    Foo.h
    Bar.h
)

set(Sources
    Foo.cpp
    Bar.cpp
)

add_library(MyLib SHARED ${Headers} ${Sources})
```

```cmake
# 禁止：直接在 add_library 里写文件列表
add_library(MyLib SHARED Foo.h Foo.cpp Bar.h Bar.cpp)
```

## 变量命名
- 头文件：`Headers`
- 源文件：`Sources`
- `Headers` 列在 `add_library` / `add_executable` 中，便于 IDE 分组显示

## DLL 导出宏
- 每个 SHARED 库有自己的导出宏（如 `GRINDING_UI_EXPORT`）
- 宏定义在对应的 `GrindingXxxExport.h` 中
- 全 inline 的类/结构体不需要导出宏
