# Pinocchio Kinematics Lite

[English](README.md) | [中文](README_zh-CN.md)

一个轻量级、基于 Pinocchio 的正逆运动学与雅可比库，适用于 URDF 机器人臂，支持内置 robot profile 和自定义 URDF。

## 概述

Pinocchio Kinematics Lite 是一个独立的运动学库。它通过 Pinocchio 加载 URDF 机器人臂，并提供简洁的 Python API，用于正运动学、数值逆运动学、连杆雅可比矩阵、关节限位以及随机关节采样。

本包刻意保持与厂商控制栈的独立性：

- 无 pyAgxArm 运行时依赖
- 无 AgileX SDK 依赖
- 无遥操作服务器、客户端、CAN 传输或硬件控制层
- 通用的 `PinocchioKinematics` 支持自定义 URDF 机械臂
- 内置 profile 注册表，覆盖 Nero、Franka Panda、Franka Panda + Robotiq 以及 ARX R5

## 适用范围

本库面向由 URDF 描述的固定基座串联机械臂。适用于离线运动学检查、简单 IK 目标、基准测试，以及已有自有机器人控制层的应用场景。

本库不是运动规划器、碰撞检测器、硬件控制器、遥操作系统或完整的动力学框架。浮动机座、仿型关节耦合、夹爪力学以及闭链机器人不在当前高层 API 的覆盖范围内。多分支 URDF 应为每条需要求解的主动链注册单独的 profile。

## 内置 Profiles

内置资源建议通过 `create_robot_kinematics(profile_name)` 创建：

| Profile 名称 | 默认末端 frame | 主动关节 |
|---|---|---|
| `nero` | `link7` | `joint1` ... `joint7` |
| `franka_panda` | `link7` | `joint1` ... `joint7` |
| `franka_panda_robotiq` | `robotiq_arg2f_base_link` | 只包含 Panda 本体 7 个关节 |
| `arx_r5` | `right_link6` | `right_joint1` ... `right_joint6` |
| `arx_r5_left` | `left_link6` | `left_joint1` ... `left_joint6` |

`arx_r5` 默认使用双臂 URDF 里的右臂；左臂使用 `arx_r5_left`。CLI 和工厂函数也接受 `franka-panda`、`franka-panda-robotiq`、`arx_r5_right` 等别名。

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

## 快速开始：内置 Profile

```python
import numpy as np
from pinocchio_kinematics_lite import create_robot_kinematics

kin = create_robot_kinematics("franka_panda")
q = np.zeros(len(kin.list_joints()))

pose = kin.forward_kinematics(q)
J = kin.jacobian(q)
result = kin.inverse_kinematics(pose, q_init=q)

print(result.success)
print(result.q)
```

`NeroKinematics` 仍保留为兼容入口；新的内置机器人通过 profile 注册表创建。URDF 解析顺序为：

1. 显式指定的 `urdf_path`
2. 对应 profile 的环境变量，例如 `NERO_URDF_PATH`、`FRANKA_PANDA_URDF_PATH`、`FRANKA_PANDA_ROBOTIQ_URDF_PATH` 或 `ARX_R5_URDF_PATH`
3. `pinocchio_kinematics_lite/assets/...` 下的包内置资源

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

## 运行时使用流程

`PinocchioKinematics` 刻意保持与遥操作和硬件控制栈解耦。运行时应用应在运动学对象外部维护自己的关节状态缓存、安全检查和命令下发层。

通用流程如下：

1. 解析 URDF，并选择末端 frame、主动关节和可选关节限位。
2. 创建 `PinocchioKinematics`。
3. 读取或选择初始关节向量 `q_seed`。
4. 将每一帧目标位姿转换成 IK 支持的输入格式。对于 `[x, y, z, roll, pitch, yaw]` 形式的扁平目标，可使用 `pose_matrix_from_xyz_rpy()`。
5. 如果命令目标是 TCP/tool frame，而不是 URDF 里的末端 frame，则用 `T_world_ee = T_world_tcp @ inv(T_ee_tcp)` 转换到 URDF 末端目标。
6. 调用 `inverse_kinematics(..., q_init=q_seed)`，检查返回的 `IKResult`，执行应用层安全检查，然后用被接受的命令更新 `q_seed`。

伪代码如下：

```python
import os
import numpy as np
from pinocchio_kinematics_lite import (
    DEFAULT_NERO_JOINT_NAMES,
    PinocchioKinematics,
    get_robot_urdf_path,
    pose_matrix_from_xyz_rpy,
)

urdf_path = get_robot_urdf_path("nero", os.getenv("NERO_URDF_PATH"))
kin = PinocchioKinematics(
    urdf_path=urdf_path,
    end_effector_frame="link7",
    active_joint_names=DEFAULT_NERO_JOINT_NAMES,
    joint_limits=joint_limits,
)

q_seed = read_current_joint_state()  # 由应用提供

while control_loop_is_running():
    target_pose6 = receive_target_pose()  # [x, y, z, roll, pitch, yaw]
    T_world_target = pose_matrix_from_xyz_rpy(target_pose6[:3], target_pose6[3:])

    if target_is_tcp_frame:
        T_ee_tcp = pose_matrix_from_xyz_rpy(tcp_offset[:3], tcp_offset[3:])
        T_world_target = T_world_target @ np.linalg.inv(T_ee_tcp)

    result = kin.inverse_kinematics(
        T_world_target,
        q_init=q_seed,
        max_iters=60,
        pos_tol=1e-4,
        ori_tol=5e-3,
    )
    if not result.success:
        handle_ik_failure(result)
        continue

    q_cmd = kin.clip_to_joint_limits(result.q)
    q_cmd = limit_joint_step(q_seed, q_cmd)  # 可选的应用层安全限制
    send_joint_target(q_cmd)  # 应用自行实现
    q_seed = q_cmd
```

当目标以 6D 扁平 pose 表示时，请先用 `pose_matrix_from_xyz_rpy()` 转为矩阵再调用 `inverse_kinematics()`。IK API 支持 4x4 矩阵、pose 字典、Pinocchio 风格 SE3 对象，或 `(position, orientation)` 元组。

## 基准测试

内置 profiles：

```bash
python benchmarks/benchmark_ik.py --robot-profile nero --num-samples 100
python benchmarks/benchmark_ik.py --robot-profile franka_panda --num-samples 100
python benchmarks/benchmark_ik.py --robot-profile franka_panda_robotiq --num-samples 100
python benchmarks/benchmark_ik.py --robot-profile arx_r5 --num-samples 100
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

## 基准对比：多机器人求解器性能

本节总结了使用 PinocchioKinematics 在不同类型机械臂上的 IK 求解器性能。所有基准测试均使用 1,000 和 10,000 个随机样本进行，测量成功率、延迟、精度和迭代统计。

### 性能总结（1,000 样本）

| 机器人 Profile | 自由度 | 成功率 | 平均延迟 (ms) | 平均迭代次数 | 平均位置误差 (m) | 平均姿态误差 (rad) |
|---|---|---|---|---|---|---|
| `arx_r5` | 6 | 100.0% | 0.095 | 2.94 | 2.68e-05 | 7.00e-05 |
| `franka_panda` | 7 | 100.0% | 0.097 | 3.16 | 1.77e-05 | 3.04e-05 |
| `franka_panda_robotiq` | 7 | 100.0% | 0.099 | 3.18 | 1.71e-05 | 2.45e-05 |
| `nero` | 7 | 99.8% | 0.099 | 3.26 | 1.75e-05 | 3.86e-05 |

### 性能总结（10,000 样本）

| 机器人 Profile | 自由度 | 成功率 | 平均延迟 (ms) | 平均迭代次数 | 平均位置误差 (m) | 平均姿态误差 (rad) |
|---|---|---|---|---|---|---|
| `arx_r5` | 6 | 100.0% | 0.092 | 2.93 | 2.64e-05 | 6.69e-05 |
| `franka_panda` | 7 | 99.96% | 0.096 | 3.17 | 1.65e-05 | 2.83e-05 |
| `franka_panda_robotiq` | 7 | 99.96% | 0.098 | 3.17 | 1.66e-05 | 2.75e-05 |
| `nero` | 7 | 99.91% | 0.097 | 3.20 | 1.79e-05 | 4.23e-05 |

### 关键观察

- **一致的性能表现**：所有机器人类型均实现亚毫秒级平均延迟（0.092–0.099 ms），展现了出色的实时性能。
- **成功率**：ARX R5 在两个测试集中均达到 100% 成功率，而 7 自由度机器人保持 99.8–100% 的成功率，超时情况极少。
- **自由度影响**：6 自由度的 ARX R5 显示的迭代次数略低于 7 自由度机器人（2.93–2.94 vs 3.16–3.26），反映了逆运动学问题的复杂性降低。
- **精度**：所有机器人的位置误差保持一致的低水平（1.65–2.68e-05 m），姿态误差在 2.45–7.00e-05 rad 范围内。
- **可扩展性**：从 1,000 个样本扩展到 10,000 个样本时，性能特征保持稳定，表明求解器在不同工作负载下具有稳健的行为。

## 基准对比：Legacy Solver vs Pinocchio Solver

本对比基于 AgileX Nero 7-DoF 机械臂的正运动学生成的可达目标与连续轨迹目标。它衡量了严格精度与延迟之间的权衡，并展示了 `PinocchioKinematics` 提供的诊断信息与通用 URDF 架构优势。传统 `Solver` 的数值在此作为参考；此基准测试并不声称某个求解器在各方面都更优。

### 单目标 IK，1,000 采样点

| 指标 | Legacy Solver | PinocchioKinematics |
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

| 指标 | Legacy Solver | PinocchioKinematics |
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

| 指标 | Legacy Solver | PinocchioKinematics |
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
- `PinocchioKinematics` 在这些基准测试中的平均延迟快约两个数量级。
- `PinocchioKinematics` 提供结构化诊断信息（`iterations`、`reason`、`best_q`、`last_q`、`solve_time_ms`）。
- `PinocchioKinematics` 是与本仓库通用 URDF 架构对齐的实现。
- `PinocchioKinematics` 极少数单目标失败案例属于严格阈值下的最大迭代次数耗尽，而非明显发散的解。
- 在记录的失败案例中，残余位置误差通常在 1e-05 至 4e-05 m 范围内，旋转误差极小，因此部署时用户可根据硬件精度和控制需求选择放宽容差。
- 在 1,000 步连续轨迹基准测试中，两个求解器均达到 100% 成功率和零构型跳变，而 `PinocchioKinematics` 的延迟显著更低。

当前基准测试支持以下论断：**`PinocchioKinematics` 更适合本仓库的实时性、可诊断性和通用 URDF 使用场景，而传统 `Solver` 在严格阈值成功率和面向 Nero 特定 FK 目标时的机器精度位姿恢复方面仍然更强。** 它并不声称 `PinocchioKinematics` 在各方面都优于所有求解器。

## 厂商独立性

本仓库主体为位于 `src/pinocchio_kinematics_lite` 下的通用包，外加测试、示例、基准测试、文档以及内置 URDF/网格资源。核心包仅依赖 Pinocchio、NumPy 以及 Python 标准库模块。它不导入 pyAgxArm、AgileX SDK 模块、zerorpc、遥操作代码、CAN 代码或硬件驱动程序。

## 添加 Profile

内置 robot-profile 名称集中注册在 `src/pinocchio_kinematics_lite/profiles/registry.py`。新增机器人时，在 `_PROFILE_DEFINITIONS` 里添加一个 `RobotProfile`，填写 URDF 资源路径、默认末端 frame、主动关节名、可选 root frame、可选关节限位覆盖和别名，并确认 `pyproject.toml`/`MANIFEST.in` 会打包对应 URDF 与 mesh。
