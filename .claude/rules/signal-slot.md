# 信号槽规范

## 防循环原则
- 双向数据绑定时，用 `blockSignals(true/false)` 或 `bool updating_` 标志防止无限循环
- 模式：写入 → guard on → set value → guard off

```cpp
// 示例：SetJointAngles 不应触发 JointAnglesChanged
void SetJointAngles(const Q& angles) {
    spinbox->blockSignals(true);
    spinbox->setValue(angles[i]);
    spinbox->blockSignals(false);
}
```

## UI 分层原则
- UI 组件（JogPanel 等）只做信号转发，不含业务逻辑
- MainWindow 作为 orchestrator 连接信号，调用算法模块
- 算法结果通过 public slot 回传给 UI 组件

## connect 写法
```cpp
// 推荐：函数指针形式
connect(sender, &Sender::Signal, receiver, &Receiver::Slot);

// Lambda 用于简单逻辑
connect(sender, &Sender::Signal, this, [this]() { ... });
```
