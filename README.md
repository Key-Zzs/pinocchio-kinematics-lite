# Pinocchio Kinematics Lite

A lightweight Pinocchio-based FK/IK/Jacobian library for URDF robot arms, with built-in support for the AgileX Nero 7-DoF arm.

## Overview

This is a standalone Pinocchio-based kinematics library for URDF robot arms. It provides a generic `PinocchioKinematics` class for custom URDFs and a built-in `NeroKinematics` profile for the AgileX Nero 7-DoF arm.

Core FK, IK, Jacobian, and joint-limit utilities do not require `pyAgxArm`, the official AgileX SDK, `zerorpc`, teleop servers, or hardware communication code.

## Features

- URDF-based robot model loading
- Forward kinematics
- Numerical inverse kinematics
- Frame Jacobian computation
- Joint-limit checking and clipping
- Random joint sampling
- Structured `IKResult` diagnostics
- Custom URDF support
- Built-in AgileX Nero 7-DoF profile
- Benchmark and debug scripts

## Scope

The library is aimed at fixed-base serial robot arms described by URDF. By default, active joints are scalar revolute/prismatic joints from the Pinocchio model. You can also pass `active_joint_names` explicitly.

This is not a motion planner, collision checker, hardware control SDK, teleop system, LeRobot training stack, or full dynamics framework. Floating bases, mimic joints, gripper coupling, and closed chains are not fully supported by the high-level API.

## Installation

Create an environment with Python 3.10+ and install Pinocchio. The most reliable route is conda-forge:

```bash
conda install -c conda-forge pinocchio eigenpy -y
pip install -e ".[test]"
```

Some Linux environments can use the cmeel wheels instead:

```bash
pip install pin
pip install -e ".[test]"
```

## Quick Start: Built-In Nero

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

`NeroKinematics()` loads the URDF in this order:

1. explicit `urdf_path`
2. `NERO_URDF_PATH`
3. bundled `assets/nero/nero_description.urdf`

It does not load URDFs from `pyAgxArm`, SDK install paths, or user-specific absolute fallback paths.

## Quick Start: Custom URDF

```python
import numpy as np
from pinocchio_kinematics_lite import PinocchioKinematics

kin = PinocchioKinematics(
    urdf_path="path/to/your_robot.urdf",
    end_effector_frame="tool0",
    active_joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
)

q = np.zeros(6)
pose = kin.forward_kinematics(q)
result = kin.inverse_kinematics(pose, q_init=q)
```

Supported target pose inputs include a 4x4 homogeneous matrix, `pinocchio.SE3`, a dict with `position` plus `quaternion` or `rpy`, and `(position, orientation)` tuples. Helpers are available as `pose_matrix_from_xyz_quat`, `pose_matrix_from_xyz_rpy`, `xyz_quat_from_pose_matrix`, and `xyz_rpy_from_pose_matrix`.

## Benchmark

```bash
pytest tests -v

python benchmarks/benchmark_ik.py \
  --robot-profile nero \
  --num-samples 1000 \
  --output results/ik_benchmark_1000.json \
  --log-failures results/ik_failures_1000.jsonl

python benchmarks/benchmark_trajectory_continuity.py \
  --robot-profile nero \
  --num-samples 300 \
  --output results/trajectory_benchmark_300.json
```

Custom URDF benchmark:

```bash
python benchmarks/benchmark_ik.py \
  --urdf-path path/to/robot.urdf \
  --end-effector-frame tool0 \
  --num-samples 200
```

## Vendor Independence

The core package imports only Pinocchio, NumPy, and the Python standard library. It does not import `pyAgxArm`, AgileX SDK modules, `zerorpc`, teleop clients/servers, or hardware interfaces. Optional vendor SDK integration should live outside the core package, for example in `examples/`, `adapters/`, or `docs/`.
