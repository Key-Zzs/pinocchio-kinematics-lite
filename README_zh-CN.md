# Pinocchio Kinematics Lite

[English](README.md) | [中文](README_zh-CN.md)

一个轻量级、基于 Pinocchio 的正逆运动学与雅可比库，适用于 URDF 机器人臂，内置支持 AgileX Nero 7-DoF 机械臂。

## 概述

Pinocchio Kinematics Lite 是一个独立的运动学库。它通过 Pinocchio 加载 URDF 机器人臂，并提供简洁的 Python API，用于正运动学、数值逆运动学、连杆雅可比矩阵、关节限位以及随机关节采样。

本包刻意保持与厂商控制栈的独立性：

- 无 pyAgxArm 运行时依赖
- 无 AgileX SDK 依赖
- 无遥操作服务器、客户端、CAN 传输或硬件控制层
- 通用的 `PinocchioKinematics` 支持自定义 URDF 机械臂
- 内置 `NeroKinematics` 配置文件，适配 AgileX Nero 7-DoF 机械臂

## 适用范围

本库面向由 URDF 描述的固定基座串联机械臂。适用于离线运动学检查、简单 IK 目标、基准测试，以及已有自有机器人控制层的应用场景。

本库不是运动规划器、碰撞检测器、硬件控制器、遥操作系统或完整的动力学框架。浮动机座、仿型关节耦合、夹爪力学以及闭链机器人不在当前高层 API 的覆盖范围内。

## 安装

使用 Python 3.10 或更高版本。请先安装 Pinocchio；通过 conda-forge 安装是最可靠的途径：

```bash
conda install -c conda-forge pinocchio eigenpy -y
pip install -e ".[test]"
```

部分 Linux 环境可使用 cmeel 分发包：

```bash
pip install pin
pip install -e ".[test]"
```

包元数据将运行时依赖保持在最小范围，不强制依赖特定 Pinocchio 分发包，因为不同平台的 Pinocchio 打包方式存在差异。

## 快速开始：Nero

```python
import numpy as np
from pinocchio_kinematics_lite import NeroKinematics

kin = NeroKinematics()
q = np.zeros(7)

pose = kin.forward_kinematics(q)
J = kin.jacobian(q)
result = kin.inverse_kinematics(pose, q_init=q)

print(result.success)
print(result.q)
```

`NeroKinematics` 是对 `PinocchioKinematics` 的薄封装。它按以下顺序解析 Nero URDF：

1. 显式指定的 `urdf_path`
2. `NERO_URDF_PATH` 环境变量
3. 包内置资源：`pinocchio_kinematics_lite/assets/nero/nero_description.urdf`

## 快速开始：自定义 URDF

```python
import numpy as np
from pinocchio_kinematics_lite import PinocchioKinematics

kin = PinocchioKinematics(
    urdf_path="path/to/my_robot.urdf",
    end_effector_frame="tool0",
)

q = np.zeros(len(kin.list_joints()))
pose = kin.forward_kinematics(q)
J = kin.jacobian(q)
result = kin.inverse_kinematics(pose, q_init=q)
```

`inverse_kinematics()` 返回一个 `IKResult`，包含 `success`、`q`、`position_error`、`orientation_error`、`iterations`、`solve_time_ms`、`reason`、`best_q` 和 `last_q`。

## 基准测试

Nero 配置文件：

```bash
python benchmarks/benchmark_ik.py --robot-profile nero --num-samples 100
python benchmarks/benchmark_trajectory_continuity.py --robot-profile nero --num-samples 300
```

自定义 URDF：

```bash
python benchmarks/benchmark_ik.py \
  --urdf-path path/to/robot.urdf \
  --end-effector-frame tool0 \
  --num-samples 100

python benchmarks/benchmark_trajectory_continuity.py \
  --urdf-path path/to/robot.urdf \
  --end-effector-frame tool0 \
  --num-samples 300
```

## 基准对比：Legacy Solver vs Pinocchio Solver

本对比基于 AgileX Nero 7-DoF 机械臂的正运动学生成的可达目标与连续轨迹目标。它衡量了严格精度与延迟之间的权衡，并展示了 `Pinocchio_Solver` 提供的诊断信息与通用 URDF 架构优势。传统 `Solver` 的数值在此作为参考；此基准测试并不声称某个求解器在各方面都更优。

### 单目标 IK，1,000 采样点

| 指标 | Legacy Solver | Pinocchio_Solver |
|---|---|---|
| 成功率 | 100.0% | 99.7% |
| 超时率 | 0.0% | 0.3% |
| 关节限位违规率 | 0.0% | 0.0% |
| 平均延迟 (ms) | 17.28 | 0.10 |
| P95 延迟 (ms) | 18.18 | 0.18 |
| P99 延迟 (ms) | 18.37 | 0.62 |
| 最大延迟 (ms) | 23.32 | 1.61 |
| 平均位置误差 (m) | 1.26e-15 | 3.11e-06 |
| 最大位置误差 (m) | 3.73e-13 | 9.99e-06 |
| 平均姿态误差 (rad) | 4.22e-09 | 9.06e-07 |
| 最大姿态误差 (rad) | 2.98e-08 | 2.83e-05 |
| 平均迭代次数 | — | 4.4 |
| 最大迭代次数 | — | 80 |

### 单目标 IK，10,000 采样点

| 指标 | Legacy Solver | Pinocchio_Solver |
|---|---|---|
| 成功率 | 100.0% | 99.65% |
| 超时率 | 0.0% | 0.35% |
| 关节限位违规率 | 0.0% | 0.0% |
| 平均延迟 (ms) | 18.09 | 0.10 |
| P95 延迟 (ms) | 19.01 | 0.18 |
| P99 延迟 (ms) | 19.25 | 0.75 |
| 最大延迟 (ms) | 38.44 | 1.66 |
| 平均位置误差 (m) | 1.34e-15 | 3.02e-06 |
| 最大位置误差 (m) | 3.34e-12 | 1.00e-05 |
| 平均姿态误差 (rad) | 4.06e-09 | 8.63e-07 |
| 最大姿态误差 (rad) | 3.65e-08 | 7.21e-05 |
| 平均迭代次数 | — | 4.4 |
| 最大迭代次数 | — | 80 |

### 连续轨迹 IK，1,000 步

| 指标 | Legacy Solver | Pinocchio_Solver |
|---|---|---|
| 成功率 | 100.0% | 100.0% |
| 超时率 | 0.0% | 0.0% |
| 关节限位违规率 | 0.0% | 0.0% |
| 构型跳变次数 | 0 | 0 |
| 构型跳变率 | 0.0% | 0.0% |
| 平均关节步长范数 (rad) | 0.037 | 0.035 |
| P95 关节步长范数 (rad) | 0.054 | 0.052 |
| 最大关节步长范数 (rad) | 0.079 | 0.067 |
| 平均延迟 (ms) | 12.61 | 0.08 |
| P95 延迟 (ms) | 18.82 | 0.08 |
| P99 延迟 (ms) | 18.89 | 0.09 |
| 最大延迟 (ms) | 19.06 | 0.35 |
| 平均位置误差 (m) | 2.27e-16 | 1.58e-06 |
| 最大位置误差 (m) | 7.66e-16 | 9.74e-06 |
| 平均姿态误差 (rad) | 4.83e-09 | 2.40e-07 |
| 最大姿态误差 (rad) | 2.98e-08 | 4.66e-05 |
| 平均迭代次数 | — | 3.0 |
| 最大迭代次数 | — | 4 |

### 结果解读

- 传统 `Solver` 在 FK 生成的目标上实现了 100% 严格阈值成功率和接近机器精度的位姿恢复。
- 传统 `Solver` 延迟较高，单目标 IK 平均延迟约 17–18 ms，轨迹基准测试平均延迟约 12.6 ms。
- `Pinocchio_Solver` 在这些基准测试中的平均延迟快约两个数量级。
- `Pinocchio_Solver` 提供结构化诊断信息（`iterations`、`reason`、`best_q`、`last_q`、`solve_time_ms`）。
- `Pinocchio_Solver` 是与本仓库通用 URDF 架构对齐的实现。
- `Pinocchio_Solver` 极少数单目标失败案例属于严格阈值下的最大迭代次数耗尽，而非明显发散的解。
- 在记录的失败案例中，残余位置误差通常在 1e-05 至 4e-05 m 范围内，旋转误差极小，因此部署时用户可根据硬件精度和控制需求选择放宽容差。
- 在 1,000 步连续轨迹基准测试中，两个求解器均达到 100% 成功率和零构型跳变，而 `Pinocchio_Solver` 的延迟显著更低。

当前基准测试支持以下论断：**`Pinocchio_Solver` 更适合本仓库的实时性、可诊断性和通用 URDF 使用场景，而传统 `Solver` 在严格阈值成功率和面向 Nero 特定 FK 目标时的机器精度位姿恢复方面仍然更强。** 它并不声称 `Pinocchio_Solver` 在各方面都优于所有求解器。

## 厂商独立性

本仓库主体为位于 `src/pinocchio_kinematics_lite` 下的通用包，外加测试、示例、基准测试、文档以及内置的 Nero URDF/网格文件。核心包仅依赖 Pinocchio、NumPy 以及 Python 标准库模块。它不导入 pyAgxArm、AgileX SDK 模块、zerorpc、遥操作代码、CAN 代码或硬件驱动程序。
