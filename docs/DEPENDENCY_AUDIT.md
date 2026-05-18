# Dependency Audit

## Removed Content

This repository was cleaned into a standalone kinematics library. The following legacy/vendor/hardware content was removed from the repository body:

- `pyAgxArm/`
- `nero/`
- `docs/effector/`
- `docs/nero/`
- `docs/piper/`
- `docs/can_user.md`
- `docs/Q&A.md`
- `docs/ubuntu_24_04_pip_install.md`
- `README_pyAgxArm.md`
- `requirements.txt`
- `rm_tmp.sh`
- `results/`
- all checked-in `__pycache__/` directories and `*.pyc` files

The removed tree included CAN communication code, hardware drivers, teleop server/client code, old Nero-specific kinematics experiments, SDK examples, vendor API docs, and generated runtime artifacts.

## Kept Robot Resources

Bundled robot assets remain only as built-in robot profiles:

- `src/pinocchio_kinematics_lite/assets/nero/nero_description.urdf`
- `src/pinocchio_kinematics_lite/assets/nero/meshes/`
- `src/pinocchio_kinematics_lite/assets/franka_panda/`
- `src/pinocchio_kinematics_lite/assets/arx_r5/`
- `src/pinocchio_kinematics_lite/profiles/registry.py`
- `src/pinocchio_kinematics_lite/profiles/nero.py`
- `examples/quick_start_nero.py`
- `tests/test_nero_profile.py`
- `tests/test_robot_profile_registry.py`

Bundled URDFs use package-local mesh paths. No fallback path points into a vendor SDK tree or a user-specific absolute directory.

## Core Dependency Boundary

The importable package under `src/pinocchio_kinematics_lite/` is limited to:

- `pinocchio`
- `numpy`
- Python standard-library modules such as `pathlib`, `dataclasses`, `typing`, `time`, and `importlib.resources`

Test code uses `pytest`. Benchmarks use only the package API, NumPy, and standard-library modules.

The core package must not contain imports or runtime hooks for pyAgxArm, AgileX SDK modules, zerorpc, teleop code, CAN drivers, hardware methods such as `get_tcp_pose()`, `get_joint_angles()`, `move_j()`, `move_js()`, or SDK response assumptions such as `.msg`.

## Robot Profile Loading Priority

Built-in profiles resolve URDFs in this order:

1. explicit `urdf_path`
2. the profile-specific environment variable, such as `NERO_URDF_PATH`, `FRANKA_PANDA_URDF_PATH`, `FRANKA_PANDA_ROBOTIQ_URDF_PATH`, or `ARX_R5_URDF_PATH`
3. bundled package asset under `pinocchio_kinematics_lite/assets/...`

For the bundled profile, the public `urdf_path` and `resolved_urdf_path` identify the package asset. The internal filesystem path used to initialize Pinocchio is also available as `filesystem_urdf_path`.

## Verification

Run:

```bash
python -m compileall -q src tests benchmarks examples

python - <<'PY'
import sys
import pinocchio_kinematics_lite as pkl
print("import ok")
print("pyAgxArm" in sys.modules)
print("zerorpc" in sys.modules)
PY

pytest tests -v

grep -R "pyAgxArm\|AgxArm\|agilex\|zerorpc\|teleop\|get_tcp_pose\|get_joint_angles\|move_j\|move_js\|servo_p_OL\|\.msg\|asserts/agx_arm_urdf\|/home/" -n src/pinocchio_kinematics_lite
```

The last command should produce no output for the core package.

## Why Robots Are Profiles

The package is not a robot-specific SDK. Built-in robot support is intentionally small: URDF/mesh asset bundles plus profile entries that select default joint names, end-effector frames, root frames when needed, and bundled URDF paths before delegating to `PinocchioKinematics`. All FK, IK, Jacobian, limit, and transform behavior lives in the generic core.
