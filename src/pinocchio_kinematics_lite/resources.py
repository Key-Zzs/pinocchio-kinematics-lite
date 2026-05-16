"""Package resource loading helpers."""

from __future__ import annotations

import os
from importlib import resources
from pathlib import Path


def package_resource_path(*parts: str) -> Path:
    """Return a filesystem path for a package resource."""
    if hasattr(resources, "files"):
        root = resources.files("pinocchio_kinematics_lite")
        resource = root.joinpath(*parts)
        return Path(resource)
    return Path(__file__).resolve().parent.joinpath(*parts)


def get_nero_urdf_path(explicit_path: str | os.PathLike[str] | None = None) -> Path:
    """Resolve the Nero URDF path.

    Priority:
    1. explicit ``urdf_path``
    2. ``NERO_URDF_PATH``
    3. bundled ``assets/nero/nero_description.urdf``
    """
    if explicit_path is not None:
        candidate = Path(explicit_path).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"Nero URDF does not exist: {candidate}")
        return candidate

    env_path = os.getenv("NERO_URDF_PATH")
    if env_path:
        candidate = Path(env_path).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"NERO_URDF_PATH does not exist: {candidate}")
        return candidate

    bundled = package_resource_path("assets", "nero", "nero_description.urdf")
    if not bundled.is_file():
        raise FileNotFoundError(f"Bundled Nero URDF is missing: {bundled}")
    return bundled
