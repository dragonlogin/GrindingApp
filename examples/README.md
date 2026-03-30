# KDL 学习示例

## 文件说明

- `kdl_ik_test.cpp` - KDL 正逆运动学测试程序（独立示例）
- `kdl_demo.cpp` - KDL 完整教程（包含 5 个示例）

## 编译

在项目根目录构建：

```bash
# 配置
cmake -S . -B build -DVCPKG_MANIFEST_INSTALL=OFF

# 构建
cmake --build build --config Release
```

编译后的可执行文件位于：`build/examples/Release/kdl_ik_test.exe`

## 运行

```bash
cd build/examples/Release
kdl_ik_test.exe
```

## 示例内容

### kdl_ik_test.cpp

测试 ABB IRB2400 机器人的正逆运动学：

1. **测试 1**: 零位姿 (所有关节 0°)
   - 验证 FK 和 IK 的基本功能
   
2. **测试 2**: 非零位姿
   - 测试任意关节角度的 FK 和 IK
   - 计算位置误差

3. **测试 3**: 多个随机位姿
   - 批量测试不同位形
   - 统计成功率

### 关键 API

```cpp
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

// 1. 构建运动学链
KDL::Chain chain;
chain.addSegment(...);

// 2. 创建正运动学求解器
KDL::ChainFkSolverPos_recursive fk_solver(chain);

// 3. 创建逆运动学求解器
KDL::ChainIkSolverVel_pinv ik_vel_solver(chain);
KDL::ChainIkSolverPos_NR_JL ik_solver(chain, fk_solver, ik_vel_solver);

// 4. 正运动学
KDL::JntArray q;
KDL::Frame result;
fk_solver.JntToCart(q, result);

// 5. 逆运动学
KDL::JntArray q_init, q_out;
ik_solver.CartToJnt(q_init, target_frame, q_out);
```

## 参考资料

- **KDL 官方文档**: https://docs.orocos.org/kdl/
- **GitHub 仓库**: https://github.com/orocos/orocos_kinematics_dynamics
- **用户手册**: https://docs.orocos.org/kdl/master/

## 机器人模型

- **IRB2400**: `model/robot/IRB2400/`
- **IRB140**: `model/robot/IRB140/`
