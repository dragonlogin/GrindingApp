# Algorithm Agent（算法开发）

- **职责**：运动学（FK/IK）、路径规划、碰撞检测等数学算法
- **范围**：`src/kinematics/`、未来的 `src/planning/`
- **规范**：纯计算，不依赖 Qt 和 OCCT；输入输出用 `nl::utils::Q`、`nl::utils::Vector3d`、`Eigen` 类型
- **验证**：每个公开函数必须附带 QTest 用例，覆盖正常值、边界值、退化情况
