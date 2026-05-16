from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "pinocchio_kinematics_lite"
FORBIDDEN_FRAGMENTS = (
    "pyAgxArm",
    "AgxArm",
    "agilex",
    "zerorpc",
    "teleop",
    "get_tcp_pose",
    "get_joint_angles",
    "move_j",
    "move_js",
    "servo_p_OL",
    ".msg",
    "asserts/agx_arm_urdf",
    "/home/",
)


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


def test_nero_default_urdf_uses_package_asset(pinocchio_available, monkeypatch):
    from pinocchio_kinematics_lite import NeroKinematics

    monkeypatch.delenv("NERO_URDF_PATH", raising=False)
    kin = NeroKinematics()
    normalized = kin.urdf_path.replace("\\", "/")
    resolved = kin.resolved_urdf_path.replace("\\", "/")

    assert normalized.endswith("assets/nero/nero_description.urdf")
    assert "pyAgxArm" not in normalized
    assert "asserts/agx_arm_urdf" not in normalized
    assert f"/{'home'}/" not in normalized
    assert resolved.endswith("assets/nero/nero_description.urdf")
    assert "pyAgxArm" not in resolved
    assert "asserts/agx_arm_urdf" not in resolved
    assert f"/{'home'}/" not in resolved


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


def test_core_source_has_no_vendor_fragments():
    offenders: list[tuple[str, str]] = []
    for path in PACKAGE_ROOT.rglob("*"):
        if not path.is_file():
            continue
        if "__pycache__" in path.parts or path.suffix == ".pyc":
            continue
        text = path.read_bytes().decode("utf-8", errors="ignore")
        normalized = text.replace("\\", "/")
        for fragment in FORBIDDEN_FRAGMENTS:
            if fragment in normalized:
                offenders.append((str(path.relative_to(PACKAGE_ROOT)), fragment))

    assert offenders == []


def test_core_python_files_have_no_vendor_imports():
    forbidden_roots = {"pyAgxArm", "AgxArm", "agilex", "zerorpc"}
    offenders: list[tuple[str, str]] = []

    for path in PACKAGE_ROOT.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    root = alias.name.split(".", 1)[0]
                    if root in forbidden_roots:
                        offenders.append((str(path.relative_to(PACKAGE_ROOT)), alias.name))
            elif isinstance(node, ast.ImportFrom) and node.module:
                root = node.module.split(".", 1)[0]
                if root in forbidden_roots:
                    offenders.append((str(path.relative_to(PACKAGE_ROOT)), node.module))

    assert offenders == []
