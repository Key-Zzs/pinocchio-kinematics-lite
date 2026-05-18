# Pinocchio Kinematics Lite

[中文](README_zh-CN.md) | [English](README.md)

A lightweight Pinocchio-based FK/IK/Jacobian library for URDF robot arms, with built-in robot profiles and custom URDF support.

## Overview

Pinocchio Kinematics Lite is a standalone kinematics library. It loads URDF robot arms with Pinocchio and exposes a small Python API for forward kinematics, numerical inverse kinematics, frame Jacobians, joint limits, and random joint sampling.

The package is intentionally independent from vendor control stacks:

- no pyAgxArm runtime dependency
- no AgileX SDK dependency
- no teleop server, client, CAN transport, or hardware-control layer
- generic `PinocchioKinematics` for custom URDF arms
- built-in profile registry for Nero, Franka Panda, Franka Panda + Robotiq, and ARX R5

## Scope

This library targets fixed-base serial robot arms described by URDF. It is useful for offline kinematics checks, simple IK targets, benchmarks, and applications that already have their own robot-control layer.

It is not a motion planner, collision checker, hardware controller, teleop system, or full dynamics framework. Floating bases, mimic-joint coupling, coupled gripper mechanics, and closed-chain robots are outside the current high-level API. Multi-branch URDFs should be registered with one active chain per profile.

## Built-In Profiles

Use `create_robot_kinematics(profile_name)` for bundled robot assets:

| Profile name | Default end-effector frame | Active joints |
|---|---|---|
| `nero` | `link7` | `joint1` ... `joint7` |
| `franka_panda` | `link7` | `joint1` ... `joint7` |
| `franka_panda_robotiq` | `robotiq_arg2f_base_link` | Panda arm joints only |
| `arx_r5` | `right_link6` | `right_joint1` ... `right_joint6` |
| `arx_r5_left` | `left_link6` | `left_joint1` ... `left_joint6` |

`arx_r5` uses the right arm from the bundled dual-arm ARX URDF. Use `arx_r5_left` for the left arm. Profile aliases such as `franka-panda`, `franka-panda-robotiq`, and `arx_r5_right` are accepted by the CLI and factory.

## Installation

Use Python 3.10 or newer. Install Pinocchio first; conda-forge is the most reliable route:

```bash
conda install -c conda-forge pinocchio eigenpy -y
pip install -e ".[test]"
```

Some Linux environments can use the cmeel wheels:

```bash
pip install pin
pip install -e ".[test]"
```

The package metadata keeps runtime dependencies minimal and does not force a Pinocchio wheel, because Pinocchio packaging differs by platform.

## Quick Start: Built-In Profile

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

`NeroKinematics` remains as a compatibility wrapper over `PinocchioKinematics`; new built-in robots use the profile registry. URDF resolution order is:

1. explicit `urdf_path`
2. the profile-specific environment variable, for example `NERO_URDF_PATH`, `FRANKA_PANDA_URDF_PATH`, `FRANKA_PANDA_ROBOTIQ_URDF_PATH`, or `ARX_R5_URDF_PATH`
3. bundled package asset under `pinocchio_kinematics_lite/assets/...`

## Quick Start: Custom URDF

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

`inverse_kinematics()` returns an `IKResult` with `success`, `q`, `position_error`, `orientation_error`, `iterations`, `solve_time_ms`, `reason`, `best_q`, and `last_q`.

## Runtime Usage Flow

`PinocchioKinematics` is intentionally independent of any teleop or hardware-control stack. A runtime application should keep its own joint-state cache, safety checks, and command-sending layer around the kinematics object.

General flow:

1. Resolve the URDF and choose the end-effector frame, active joints, and optional joint limits.
2. Construct `PinocchioKinematics`.
3. Read or choose an initial joint vector `q_seed`.
4. Convert each target pose to a supported IK input format. For flat `[x, y, z, roll, pitch, yaw]` targets, use `pose_matrix_from_xyz_rpy()`.
5. If the command target is a TCP/tool frame instead of the URDF end-effector frame, convert it with `T_world_ee = T_world_tcp @ inv(T_ee_tcp)`.
6. Call `inverse_kinematics(..., q_init=q_seed)`, inspect the returned `IKResult`, apply application-level safety checks, then update `q_seed` with the accepted command.

Pseudocode:

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

q_seed = read_current_joint_state()  # application-provided

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
    q_cmd = limit_joint_step(q_seed, q_cmd)  # optional application safety
    send_joint_target(q_cmd)  # application-specific
    q_seed = q_cmd
```

When the target is represented as a flat 6D pose, convert it with `pose_matrix_from_xyz_rpy()` before calling `inverse_kinematics()`. The IK API accepts a 4x4 matrix, a pose dict, a Pinocchio-like SE3 object, or `(position, orientation)` tuples.

## Benchmarks

Built-in profiles:

```bash
python benchmarks/benchmark_ik.py --robot-profile nero --num-samples 100
python benchmarks/benchmark_ik.py --robot-profile franka_panda --num-samples 100
python benchmarks/benchmark_ik.py --robot-profile franka_panda_robotiq --num-samples 100
python benchmarks/benchmark_ik.py --robot-profile arx_r5 --num-samples 100
python benchmarks/benchmark_trajectory_continuity.py --robot-profile nero --num-samples 300
```

Custom URDF:

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

## Benchmark: Multi-Robot Solver Performance

This section summarizes the IK solver performance across different robot arm types using PinocchioKinematics. All benchmarks were conducted with 1,000 and 10,000 random samples, measuring success rate, latency, accuracy, and iteration statistics.

### Performance Summary (1,000 Samples)

| Robot Profile | DOF | Success Rate | Mean Latency (ms) | Mean Iterations | Mean Position Error (m) | Mean Orientation Error (rad) |
|---|---|---|---|---|---|---|
| `arx_r5` | 6 | 100.0% | 0.095 | 2.94 | 2.68e-05 | 7.00e-05 |
| `franka_panda` | 7 | 100.0% | 0.097 | 3.16 | 1.77e-05 | 3.04e-05 |
| `franka_panda_robotiq` | 7 | 100.0% | 0.099 | 3.18 | 1.71e-05 | 2.45e-05 |
| `nero` | 7 | 99.8% | 0.099 | 3.26 | 1.75e-05 | 3.86e-05 |

### Performance Summary (10,000 Samples)

| Robot Profile | DOF | Success Rate | Mean Latency (ms) | Mean Iterations | Mean Position Error (m) | Mean Orientation Error (rad) |
|---|---|---|---|---|---|---|
| `arx_r5` | 6 | 100.0% | 0.092 | 2.93 | 2.64e-05 | 6.69e-05 |
| `franka_panda` | 7 | 99.96% | 0.096 | 3.17 | 1.65e-05 | 2.83e-05 |
| `franka_panda_robotiq` | 7 | 99.96% | 0.098 | 3.17 | 1.66e-05 | 2.75e-05 |
| `nero` | 7 | 99.91% | 0.097 | 3.20 | 1.79e-05 | 4.23e-05 |

### Key Observations

- **Consistent Performance**: All robot types achieve sub-millisecond mean latency (0.092–0.099 ms), demonstrating excellent real-time performance.
- **Success Rate**: ARX R5 achieves 100% success rate in both test sets, while 7-DoF robots maintain 99.8–100% success rates with minimal timeout occurrences.
- **DOF Impact**: The 6-DoF ARX R5 shows slightly lower iteration counts (2.93–2.94) compared to 7-DoF robots (3.16–3.26), reflecting the reduced complexity of the inverse kinematics problem.
- **Accuracy**: Position errors remain consistently low across all robots (1.65–2.68e-05 m), with orientation errors in the 2.45–7.00e-05 rad range.
- **Scalability**: Performance characteristics remain stable when scaling from 1,000 to 10,000 samples, indicating robust solver behavior under varying workloads.

## Benchmark: Legacy Solver vs Pinocchio Solver

This comparison uses FK-generated reachable targets and continuous trajectory targets for the AgileX Nero 7-DoF arm. It measures the trade-off between strict accuracy and latency, and highlights the diagnostics and generic URDF architecture available from `PinocchioKinematics`. The legacy `Solver` values are provided for context; this benchmark does not claim that one solver is universally better.

### Single-target IK, 1,000 samples

| Metric | Legacy Solver | PinocchioKinematics |
|---|---|---|
| Success rate | 100.0% | 99.7% |
| Timeout rate | 0.0% | 0.3% |
| Joint-limit violation rate | 0.0% | 0.0% |
| Mean latency (ms) | 17.28 | 0.10 |
| P95 latency (ms) | 18.18 | 0.18 |
| P99 latency (ms) | 18.37 | 0.62 |
| Max latency (ms) | 23.32 | 1.61 |
| Mean position error (m) | 1.26e-15 | 3.11e-06 |
| Max position error (m) | 3.73e-13 | 9.99e-06 |
| Mean orientation error (rad) | 4.22e-09 | 9.06e-07 |
| Max orientation error (rad) | 2.98e-08 | 2.83e-05 |
| Mean iterations | — | 4.4 |
| Max iterations | — | 80 |

### Single-target IK, 10,000 samples

| Metric | Legacy Solver | PinocchioKinematics |
|---|---|---|
| Success rate | 100.0% | 99.65% |
| Timeout rate | 0.0% | 0.35% |
| Joint-limit violation rate | 0.0% | 0.0% |
| Mean latency (ms) | 18.09 | 0.10 |
| P95 latency (ms) | 19.01 | 0.18 |
| P99 latency (ms) | 19.25 | 0.75 |
| Max latency (ms) | 38.44 | 1.66 |
| Mean position error (m) | 1.34e-15 | 3.02e-06 |
| Max position error (m) | 3.34e-12 | 1.00e-05 |
| Mean orientation error (rad) | 4.06e-09 | 8.63e-07 |
| Max orientation error (rad) | 3.65e-08 | 7.21e-05 |
| Mean iterations | — | 4.4 |
| Max iterations | — | 80 |

### Continuous trajectory IK, 1,000 steps

| Metric | Legacy Solver | PinocchioKinematics |
|---|---|---|
| Success rate | 100.0% | 100.0% |
| Timeout rate | 0.0% | 0.0% |
| Joint-limit violation rate | 0.0% | 0.0% |
| Configuration jumps | 0 | 0 |
| Configuration jump rate | 0.0% | 0.0% |
| Mean joint step norm (rad) | 0.037 | 0.035 |
| P95 joint step norm (rad) | 0.054 | 0.052 |
| Max joint step norm (rad) | 0.079 | 0.067 |
| Mean latency (ms) | 12.61 | 0.08 |
| P95 latency (ms) | 18.82 | 0.08 |
| P99 latency (ms) | 18.89 | 0.09 |
| Max latency (ms) | 19.06 | 0.35 |
| Mean position error (m) | 2.27e-16 | 1.58e-06 |
| Max position error (m) | 7.66e-16 | 9.74e-06 |
| Mean orientation error (rad) | 4.83e-09 | 2.40e-07 |
| Max orientation error (rad) | 2.98e-08 | 4.66e-05 |
| Mean iterations | — | 3.0 |
| Max iterations | — | 4 |

### Interpretation

- The legacy `Solver` achieves 100% strict-threshold success and near machine-precision pose recovery on FK-generated targets.
- The legacy `Solver` is much slower, around 17–18 ms mean latency for single-target IK and around 12.6 ms mean latency on the trajectory benchmark.
- `PinocchioKinematics` is about two orders of magnitude faster in mean latency on these benchmarks.
- `PinocchioKinematics` provides structured diagnostics (`iterations`, `reason`, `best_q`, `last_q`, `solve_time_ms`).
- `PinocchioKinematics` is the implementation aligned with this repository's generic URDF architecture.
- The rare single-target failures for `PinocchioKinematics` are strict-threshold max-iteration cases, not obviously divergent solutions.
- In the logged failures, residual position errors are typically around 1e-05 to 4e-05 m and rotation errors are very small, so deployment users may choose a relaxed tolerance depending on hardware accuracy and control requirements.
- In the 1,000-step trajectory benchmark, both solvers reached 100% success and 0 configuration jumps, while `PinocchioKinematics` had much lower latency.

The current benchmark supports this claim: **`PinocchioKinematics` is better suited for this repository's real-time, diagnostic, and generic URDF use case, while the legacy `Solver` remains stronger in strict-threshold success rate and machine-precision pose recovery on Nero-specific FK-generated targets.** It does not claim that `PinocchioKinematics` is universally better than every solver.

## Vendor Independence

The repository body is the generic package under `src/pinocchio_kinematics_lite`, plus tests, examples, benchmarks, docs, and bundled URDF/mesh assets. The core package imports Pinocchio, NumPy, and Python standard-library modules only. It does not import pyAgxArm, AgileX SDK modules, zerorpc, teleop code, CAN code, or hardware drivers.

## Adding Profiles

Built-in robot-profile names are registered in `src/pinocchio_kinematics_lite/profiles/registry.py`. Add a `RobotProfile` entry with the URDF resource path, default end-effector frame, active joint names, optional root frame, optional joint-limit override, and optional aliases. Also make sure package data includes the new URDF and meshes.
