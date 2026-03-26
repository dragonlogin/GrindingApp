# 命名空间规范

## 两层命名空间结构

所有类必须在两层命名空间内：

| 第一层 | 第二层 | 对应目录 |
|---|---|---|
| `nl` | `core` | `src/core/` |
| `nl` | `utils` | `src/utils/` |
| `nl` | `occ` | `src/occ/` |
| `nl` | `kinematics` | `src/kinematics/` |
| `nl` | `ui` | `src/ui/` |

## 写法
```cpp
namespace nl {
namespace kinematics {

class Foo {};  // 不缩进

} // namespace kinematics
} // namespace nl
```

## 跨命名空间引用
- 用完整限定名：`nl::core::RbRobot`
- 或在 .cpp 中用 `using` 声明：`using nl::core::RbRobot;`
- 禁止 `using namespace`
