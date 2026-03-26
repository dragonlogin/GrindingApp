# 3D/OCC Agent（三维可视化）

- **职责**：OCCT 几何操作、STL/STEP 加载、坐标变换工具函数、3D 场景管理
- **范围**：`src/occ/`
- **规范**：对外接口用 `gp_Trsf`、`TopoDS_Shape` 等 OCCT 类型；内部可用 Eigen 做中间计算
- **协作**：为 UI Agent 提供显示用的 `AIS_Shape`、`AIS_Trihedron`；为 Algorithm Agent 提供几何工具
