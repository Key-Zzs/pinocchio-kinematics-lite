# Dependency Audit

## Original Vendor Dependencies

The repository originally mixed kinematics, hardware SDK code, teleop code, and robot-specific debug scripts. The main vendor/hardware dependencies found in the legacy tree were:

- `pyAgxArm`
- AgileX CAN and SDK modules
- `zerorpc`
- teleop client/server modules
- hardware methods such as `get_tcp_pose()`, `get_joint_angles()`, `move_j()`, `move_js()`, and `servo_p_OL()`
- SDK response assumptions such as `.msg`
- URDF fallback paths under `pyAgxArm/asserts/agx_arm_urdf`

## Removed From Core

The new package is under `src/pinocchio_kinematics_lite` and is the only package discovered by `pyproject.toml`. Default dependencies no longer include CAN, `zerorpc`, `pyAgxArm`, SDK modules, PyBullet, visualization stacks, or teleop code.

The legacy vendor/teleop directories may still exist in the checkout for reference, but they are not imported by the core package and are excluded from packaging.

## Core Dependency List

- `numpy`
- `pinocchio` runtime module, normally installed from conda-forge as `pinocchio`
- Python standard library modules such as `pathlib`, `dataclasses`, `typing`, `json`, `time`, and `importlib.resources`

Tests use `pytest`. Benchmarks use only standard library modules and NumPy.

## Optional Integration Boundary

Any future vendor SDK integration should live outside `src/pinocchio_kinematics_lite`, for example:

- `examples/pyagxarm_integration.py`
- `adapters/pyagxarm_adapter.py`
- `docs/integration_pyagxarm.md`

Such files must use `try/except ImportError` and give a clear message when the SDK is not installed.

## URDF Loading Priority

`NeroKinematics` resolves the URDF in this order:

1. explicit `urdf_path`
2. `NERO_URDF_PATH`
3. bundled `pinocchio_kinematics_lite/assets/nero/nero_description.urdf`

It does not fall back to `pyAgxArm/asserts/agx_arm_urdf`, SDK installation directories, or user-specific absolute paths.

## Verification

```bash
python - <<'PY'
import sys
import pinocchio_kinematics_lite as pkl
print("import ok")
print("pyAgxArm" in sys.modules)
print("zerorpc" in sys.modules)
PY

pytest tests/test_no_vendor_dependency.py -v

grep -R "pyAgxArm\|AgxArm\|agilex\|zerorpc\|get_tcp_pose\|get_joint_angles\|move_j\|move_js\|servo_p_OL\|\.msg\|asserts/agx_arm_urdf" -n src tests benchmarks examples README.md
```

The grep may intentionally find text in this audit document or optional integration examples, but it should not find vendor imports in `src/pinocchio_kinematics_lite`.
