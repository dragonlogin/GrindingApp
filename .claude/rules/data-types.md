# 数据结构要求

非 Qt 信号槽代码中，**禁止**使用以下类型：

| 禁止 | 替代 |
|---|---|
| `QList` | `std::vector` |
| `QVector` | `std::vector` |
| `QString` | `std::string` |
| `QVariant` | 具体类型 或 `std::any` |
| `QMap` | `std::map` / `std::unordered_map` |

**允许例外**：`connect()` 的信号槽参数、`QObject` 子类的 `Q_PROPERTY`

**判断规则**：这个函数是否直接服务于信号槽？否则一律用 std 类型。

**禁止过度封装**：避免不必要的复杂对象包装，保持内存布局透明。
