from __future__ import annotations

import importlib
import sys


def test_core_import_does_not_load_vendor_modules():
    for name in list(sys.modules):
        if name == "pyAgxArm" or name.startswith("pyAgxArm."):
            sys.modules.pop(name, None)
        if name == "zerorpc" or name.startswith("zerorpc."):
            sys.modules.pop(name, None)

    pkl = importlib.import_module("pinocchio_kinematics_lite")

    assert pkl is not None
    forbidden_prefixes = ("pyAgxArm", "agilex", "zerorpc")
    loaded = sorted(name for name in sys.modules if name.startswith(forbidden_prefixes))
    assert loaded == []


def test_nero_default_urdf_uses_package_or_env_asset(pinocchio_available):
    from pinocchio_kinematics_lite import NeroKinematics

    kin = NeroKinematics()
    normalized = kin.urdf_path.replace("\\", "/")

    assert normalized.endswith("assets/nero/nero_description.urdf")
    assert "pyAgxArm" not in normalized
    assert "asserts/agx_arm_urdf" not in normalized


def test_core_package_does_not_import_hardware_layers():
    import pinocchio_kinematics_lite  # noqa: F401

    forbidden_fragments = ("teleop", "server", "client", "hardware")
    loaded = [
        name
        for name in sys.modules
        if name.startswith("pinocchio_kinematics_lite")
        and any(fragment in name for fragment in forbidden_fragments)
    ]
    assert loaded == []
