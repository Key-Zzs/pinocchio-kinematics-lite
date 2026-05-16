# Pinocchio Kinematics Lite

A lightweight Pinocchio-based FK/IK/Jacobian library for URDF robot arms, with built-in support for the AgileX Nero 7-DoF arm.

## Overview

Pinocchio Kinematics Lite is a standalone kinematics library. It loads URDF robot arms with Pinocchio and exposes a small Python API for forward kinematics, numerical inverse kinematics, frame Jacobians, joint limits, and random joint sampling.

The package is intentionally independent from vendor control stacks:

- no pyAgxArm runtime dependency
- no AgileX SDK dependency
- no teleop server, client, CAN transport, or hardware-control layer
- generic `PinocchioKinematics` for custom URDF arms
- built-in `NeroKinematics` profile for the AgileX Nero 7-DoF arm

## Scope

This library targets fixed-base serial robot arms described by URDF. It is useful for offline kinematics checks, simple IK targets, benchmarks, and applications that already have their own robot-control layer.

It is not a motion planner, collision checker, hardware controller, teleop system, or full dynamics framework. Floating bases, mimic-joint coupling, gripper mechanics, and closed-chain robots are outside the current high-level API.

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

## Quick Start: Nero

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

`NeroKinematics` is a thin wrapper over `PinocchioKinematics`. It resolves the Nero URDF in this order:

1. explicit `urdf_path`
2. `NERO_URDF_PATH`
3. bundled package asset: `pinocchio_kinematics_lite/assets/nero/nero_description.urdf`

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

## Benchmarks

Nero profile:

```bash
python benchmarks/benchmark_ik.py --robot-profile nero --num-samples 100
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

## Vendor Independence

The repository body is the generic package under `src/pinocchio_kinematics_lite`, plus tests, examples, benchmarks, docs, and the bundled Nero URDF/meshes. The core package imports Pinocchio, NumPy, and Python standard-library modules only. It does not import pyAgxArm, AgileX SDK modules, zerorpc, teleop code, CAN code, or hardware drivers.
