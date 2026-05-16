# Pinocchio Kinematics Lite

[中文](README_zh-CN.md) | [English](README.md)

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

## Benchmark: Legacy Solver vs Pinocchio Solver

This comparison uses FK-generated reachable targets and continuous trajectory targets for the AgileX Nero 7-DoF arm. It measures the trade-off between strict accuracy and latency, and highlights the diagnostics and generic URDF architecture available from `Pinocchio_Solver`. The legacy `Solver` values are provided for context; this benchmark does not claim that one solver is universally better.

### Single-target IK, 1,000 samples

| Metric | Legacy Solver | Pinocchio_Solver |
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

| Metric | Legacy Solver | Pinocchio_Solver |
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

| Metric | Legacy Solver | Pinocchio_Solver |
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
- `Pinocchio_Solver` is about two orders of magnitude faster in mean latency on these benchmarks.
- `Pinocchio_Solver` provides structured diagnostics (`iterations`, `reason`, `best_q`, `last_q`, `solve_time_ms`).
- `Pinocchio_Solver` is the implementation aligned with this repository's generic URDF architecture.
- The rare single-target failures for `Pinocchio_Solver` are strict-threshold max-iteration cases, not obviously divergent solutions.
- In the logged failures, residual position errors are typically around 1e-05 to 4e-05 m and rotation errors are very small, so deployment users may choose a relaxed tolerance depending on hardware accuracy and control requirements.
- In the 1,000-step trajectory benchmark, both solvers reached 100% success and 0 configuration jumps, while `Pinocchio_Solver` had much lower latency.

The current benchmark supports this claim: **`Pinocchio_Solver` is better suited for this repository's real-time, diagnostic, and generic URDF use case, while the legacy `Solver` remains stronger in strict-threshold success rate and machine-precision pose recovery on Nero-specific FK-generated targets.** It does not claim that `Pinocchio_Solver` is universally better than every solver.

## Vendor Independence

The repository body is the generic package under `src/pinocchio_kinematics_lite`, plus tests, examples, benchmarks, docs, and the bundled Nero URDF/meshes. The core package imports Pinocchio, NumPy, and Python standard-library modules only. It does not import pyAgxArm, AgileX SDK modules, zerorpc, teleop code, CAN code, or hardware drivers.
