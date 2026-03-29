# URDF To KDL Mapping Notes

## 背景

项目当前的运动学主链已经切到：

- `URDF -> urdfdom -> KDL::Chain -> FK/IK`

显示层仍然会复用 `RbRobot` 里的 mesh / drawable 信息，但 **KDL 建链本身不应该再依赖 `RbRobot.joints` 去重建几何关系**。

## 本次踩坑

在把 `URDF -> KDL` 从旁路函数切成主流程时，曾出现过一个明显错误：

- 第 2 到第 6 关节看起来都绕着上一个轴旋转
- 连杆位置像是挂在前一个关节上
- 机器人整体会散开，关节中心明显不在正确位置

这不是 URDF 模型文件本身的问题，也不是 STL / DAE 渲染资源的问题，而是 **`URDF joint origin` 到 `KDL::Joint / KDL::Segment` 的映射写错了**。

## 错误原因

KDL 的 `Segment` 语义很容易误解。

`Segment(joint, f_tip)` 不是“把 joint 前后的所有固定变换都随便塞到 `f_tip` 里”就行，而是有严格的组合顺序。  
如果把 `parent_to_joint_origin_transform` 拆坏了，就会出现：

- 先绕错位置旋转，再平移
- 或者关节间的平移链被截断
- 最终表现成“当前关节绕上一个关节的轴转”

这次出错的直接原因是：

- 一度把 `origin`、`axis`、`f_tip` 分配错了
- 曾把 `origin frame` 只保留一部分到 `Segment`
- 导致关节轴心和连杆偏移没有按 KDL 预期组合

## 正确做法

这里必须对齐 `kdl_parser` 的标准映射思路：

1. `joint origin` 的平移部分用于关节轴心位置
2. `joint axis` 要先经过 `origin` 的旋转，转换到父坐标系
3. `Segment` 仍然保留完整的 `origin frame`
4. `offset_deg` 作为 joint offset 叠到 `KDL::Joint`

在代码里，对应实现位于：

- `src/kinematics/KdlChainBuilder.cpp`

核心逻辑是：

- `joint_origin = origin_frame.p`
- `axis_in_parent = origin_frame.M * axis_in_joint`
- `KDL::Joint(... origin, axis_in_parent, ..., offset)`
- `KDL::Segment(..., origin_frame)`

## 经验结论

- `URDF -> KDL` 映射不要靠“看起来合理”去试
- 这层一定要对齐 `kdl_parser` 的语义
- 如果出现“第 N 个关节绕第 N-1 个轴转”的现象，优先检查：
  - `joint origin` 是否放错
  - `axis` 是否忘了转到父坐标系
  - `Segment f_tip` 是否被错误截断

## 当前约束

- 运动学建链：`URDF -> KDL`
- 显示资源解析：仍可复用 `RbRobot`
- 当前项目假设单条串联链，并从 URDF joint 列表推导 `base_link / flange_link / tool_link`

后续如果要支持分支树、多个 TCP、移动底座，需要把这份假设再抽象掉。
