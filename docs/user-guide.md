# GrindingApp 使用手册

## 启动

```
build/bin/Release/GrindingApp.exe
```

---

## 基本操作

### 3D 视口交互

| 操作 | 鼠标 |
|------|------|
| 旋转 | 右键拖拽 |
| 平移 | 中键拖拽 |
| 缩放 | 滚轮 |
| 选取 | 左键点击 |

### 视图切换

菜单 **View** 或工具栏按钮：

- Front View / Top View / Side View / Isometric
- Wireframe / Shaded
- Fit All

---

## 完整工作流程

### 1. 加载机器人

**File → Load Robot** (`Ctrl+R`)

1. 点击菜单栏 **File → Load Robot**，或按 `Ctrl+R`
2. 在弹出的文件对话框中，导航到 `model/robot/IRB140/` 目录
3. 选择 `IRB140.rb.xml`，点击"打开"
4. 等待加载完成（状态栏显示机器人名称）

**验证：**
- 3D 视口中出现 IRB140 六轴机器人模型
- 左侧 **Station Manager** 出现 `IRB140` 节点，展开可见 Joint1 ~ Joint6 关节树
- 右侧 **Jog Panel** 出现 6 个关节滑块，拖动任意滑块机器人实时运动
- 尝试在 Jog Panel 的数值框中输入角度值（如 Joint1 输入 `30`），回车后机器人转到对应姿态

### 2. 加载工具

**File → Load Tool** (`Ctrl+T`)

1. 点击菜单栏 **File → Load Tool**，或按 `Ctrl+T`
2. 导航到 `model/tool/Burr/` 目录
3. 选择 `Burr.urdf`，点击"打开"

**验证：**
- 工具模型出现在机器人末端（Joint6 法兰处）
- 拖动关节滑块时，工具跟随末端一起运动
- Station Manager 中 Joint6 节点下出现 `Burr` 子节点

> 也可以加载 `model/tool/HQ2/HQ2.urdf` 作为替代工具。

### 3. 导入工件

**File → Import Workpiece** (`Ctrl+O`)

1. 点击菜单栏 **File → Import Workpiece**，或按 `Ctrl+O`
2. 在文件对话框中选择一个 STEP 格式文件（`.step` 或 `.stp`）
3. 点击"打开"，等待导入完成

**验证：**
- 工件以银白色显示在 3D 场景中
- 状态栏显示文件名和面数（如 `blade.step — 12 faces`）
- Station Manager 底部出现工件节点
- 用鼠标滚轮缩放、右键拖拽旋转，确认工件可从各角度查看

> 工件位置默认在世界原点，如果与机器人重叠，可旋转视角确认两者位置关系。

### 4. 选择加工面

**Path → Select Face**

1. 点击菜单栏 **Path → Select Face**
2. 观察状态栏提示 `"Click on a surface to select it"`
3. 将鼠标移到工件上，**左键点击**目标加工面
4. 选中的面变为**半透明青色**高亮

**验证：**
- 状态栏显示 `"Surface selected, ready to generate waypoints"`
- 只有被点击的那个面高亮，其他面保持银白色
- 如果点击到空白处或非工件区域，不会有任何变化
- 可以重新点击 **Path → Select Face** 选择其他面

**注意：** 必须先导入工件（步骤 3）才能选面，否则无可选对象。

### 5. 生成路径点

**Path → Generate Waypoints** (`Ctrl+G`)

1. 确认已选中一个加工面（步骤 4）
2. （可选）切换生成模式：**Path → Generation Mode → Grid** 或 **Planar Cut**
   - **Grid**: UV 参数网格采样，蛇形遍历路径
   - **Planar Cut**: 平行平面切割曲面，沿截交线采样路径
3. 点击 **Path → Generate Waypoints**，或按 `Ctrl+G`

**验证：**
- 选中面上出现**黄色折线**，表示生成的路径点连线
- 折线覆盖整个选中面，路径走向取决于生成模式
- 切换模式后重新生成（`Ctrl+G`），折线形状会改变
- 状态栏显示生成的路径点数量

**注意：** 如果没有选中面就点击生成，状态栏会提示错误。

### 6. 规划轨迹

在右侧 **Trajectory Editor** 面板中操作：

1. 找到右侧 Dock 区域的 **Trajectory Editor** 面板
2. 在 **Approach dist** 数值框中设置进刀距离（默认 `50` mm）
   - 进刀距离是机器人从空中接近第一个加工点的安全距离
3. 点击 **Plan** 按钮
4. 或使用菜单 **Path → Plan Trajectory**（`Ctrl+P`）
5. 等待规划完成（取决于路径点数量，通常几秒内完成）

**验证 — 表格内容：**

规划完成后表格自动填充，每行代表一个轨迹点：

| 列 | 含义 | 示例 |
|----|------|------|
| # | 点序号 | 0, 1, 2, ... |
| Type | MJ = MoveJ（关节插值），ML = MoveL（直线插值） | MJ / ML |
| X Y Z | TCP 位置（mm） | 100.0, 200.0, 50.0 |
| Rx Ry Rz | TCP 姿态（度） | 0.0, 90.0, 0.0 |
| Status | 求解状态 | OK / IK FAIL / JUMP |
| WP | 对应的 waypoint 编号 | wp0, wp1... 或空 |

**验证 — 轨迹结构：**
- 表格前半段标记为 **MJ**（MoveJ），这是从当前位姿到进刀点的关节空间运动
- 后半段标记为 **ML**（MoveL），这是沿加工路径的直线运动
- **Waypoint 行**以**粗体**显示（WP 列有值），其余为插值中间点

**验证 — 状态标记：**
- 绿/白色背景 = **OK**，逆运动学求解成功
- **红色背景** = **IK FAIL**，该点无法求解逆运动学（机器人到不了）
- **黄色背景** = **JUMP**，相邻点关节跳变 > 30°（可能存在奇异点）

### 7. 编辑轨迹（处理异常点）

当表格中存在红色或黄色异常行时，需要手动编辑：

1. **点击异常行**（红色/黄色背景的行）
   - 机器人在 3D 视口中预览该点的姿态
   - 如果是 IK FAIL，机器人保持上一个有效姿态
2. 查看表格下方的 **IK Solution** 下拉框
   - 下拉框列出该点所有可用的 IK 解（如 `Solution 0`, `Solution 1`, ...）
   - 当前使用的解已被选中
3. **切换 IK 解**：从下拉框选择另一个解
   - 机器人实时更新到新姿态
   - 状态自动重新评估（可能从 IK FAIL 变为 OK，或从 JUMP 变为 OK）
4. 逐一处理所有异常行，目标是消除所有红色和黄色标记

**验证：**
- 点击正常行（OK），机器人平滑移动到对应姿态
- 切换 IK 解后，机器人姿态明显变化（如肘部翻转）
- 处理完所有异常后，Summary 区域显示 `"Errors: 0"`
- 如果某个点确实无解（所有 IK 解都失败），可能需要调整工件位置或选择其他加工面

### 8. 回放轨迹

底部 **Playback** 面板控制轨迹动画回放：

1. 确认轨迹已规划完成（步骤 6）
2. 点击 **▶ 播放**按钮
   - 机器人从第 0 帧开始沿轨迹运动
   - 帧号标签显示当前进度（如 `"34 / 150"`）
3. 点击 **⏸ 暂停**可随时暂停
4. 点击 **⏹ 停止**回到第 0 帧
5. **拖拽进度条**可跳转到任意帧，机器人立即更新姿态
6. 调节 **Speed 滑块**控制回放速度（`0.1x` ~ `3.0x`）

| 控件 | 功能 |
|------|------|
| ▶ | 播放（从当前帧开始，播完自动回到开头） |
| ⏸ | 暂停在当前帧 |
| ⏹ | 停止并回到第 0 帧 |
| 进度条 | 拖拽跳转到任意帧 |
| Speed 滑块 | 调节速度 0.1x ~ 3.0x |

**验证：**
- 播放时机器人沿轨迹连续运动，先执行 MoveJ 到进刀点，再沿加工路径 MoveL
- 暂停后再次点击播放，从当前帧继续
- 停止后机器人回到初始姿态（第 0 帧）
- 拖拽进度条，机器人姿态实时跟随
- 调低速度到 `0.1x` 可仔细观察运动细节，调高到 `3.0x` 可快速预览

### 9. 清除与重做

- **Path → Clear Waypoints**: 清除路径点（黄色折线消失），保留选中的面
- **Path → Clear Trajectory**: 清除轨迹（表格清空、回放重置），保留路径点

**典型重做流程：**
1. 发现路径不理想 → **Clear Waypoints** → 切换生成模式 → 重新生成
2. 轨迹有太多异常 → **Clear Trajectory** → 调整进刀距离 → 重新规划
3. 选错了面 → **Clear Waypoints** → **Select Face** → 选择新面 → 重新生成

---

## 快捷键一览

| 快捷键 | 功能 |
|--------|------|
| `Ctrl+R` | 加载机器人 |
| `Ctrl+T` | 加载工具 |
| `Ctrl+O` | 导入工件 |
| `Ctrl+G` | 生成路径点 |
| `Ctrl+P` | 规划轨迹 |
| `Ctrl+Z` | 撤销 |
| `Ctrl+Y` | 重做 |
| `Ctrl+Q` | 退出 |

---

## Jog 面板

右侧 Jog Panel 提供：

- **关节控制**: 6 个滑块 + 数值框，实时拖动关节
- **TCP 位姿编辑**: 输入 X Y Z Rx Ry Rz，自动计算 IK
- **参考系切换**: Flange / Tool TCP
- **IK 多解选择**: 下拉框显示所有 IK 解，点击切换

---

## 场景树

左侧 **Station Manager** 显示层级结构：

```
Station-1
├── IRB140 (Robot)
│   ├── Joint1    30.0°
│   ├── Joint2   -45.0°
│   ├── ...
│   └── Joint6 (TCP) 0.0°
│       └── Burr (Tool)
│           └── Burr.TCP0 (Frame)
└── workpiece_name (Workpiece)
```

右键菜单：
- 关节节点 → Toggle Frame Visibility（显示/隐藏坐标系）
- 工件节点 → Generate Waypoints

---

## 快速上手示例

以下是一个从头到尾的完整操作示例：

```
1. 启动程序         build/bin/Release/GrindingApp.exe
2. 加载机器人       Ctrl+R → model/robot/IRB140/IRB140.rb.xml
3. 加载工具         Ctrl+T → model/tool/Burr/Burr.urdf
4. 导入工件         Ctrl+O → 选择一个 .step 文件
5. 选择面           Path → Select Face → 左键点击工件上的一个面
6. 生成路径点       Ctrl+G（默认 Grid 模式）
7. 规划轨迹         Trajectory Editor 面板 → Approach dist: 50 → Plan
8. 处理异常         点击红色/黄色行 → 切换 IK 解 → 直到 Errors: 0
9. 回放             底部 Playback → ▶ 播放 → 观察机器人运动
10. （可选）切换模式  Path → Generation Mode → Planar Cut → Ctrl+G → 重新规划
```

---

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 选面无反应 | 未进入选面模式 | 先点击 Path → Select Face |
| 生成路径点报错 | 未选择加工面 | 先完成步骤 4 |
| 轨迹全部 IK FAIL | 工件超出机器人工作空间 | 调整工件位置或选择更近的面 |
| 回放按钮灰色 | 未规划轨迹 | 先完成步骤 6 |
| JUMP 警告过多 | 路径经过奇异点区域 | 尝试切换生成模式或选择其他面 |
