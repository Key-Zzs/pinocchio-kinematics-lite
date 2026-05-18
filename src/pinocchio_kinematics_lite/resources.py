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


def package_resource_name(*parts: str) -> str:
    """Return the public package-resource name used in diagnostics."""
    return "pinocchio_kinematics_lite/" + "/".join(parts)


def resolve_urdf_path(
    explicit_path: str | os.PathLike[str] | None,
    *,
    env_var: str | None,
    resource_parts: tuple[str, ...],
    label: str,
) -> Path:
    """Resolve an explicit, environment, or bundled URDF path."""
    if explicit_path is not None:
        candidate = Path(explicit_path).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"{label} URDF does not exist: {candidate}")
        return candidate

    if env_var:
        env_path = os.getenv(env_var)
        if env_path:
            candidate = Path(env_path).expanduser().resolve()
            if not candidate.is_file():
                raise FileNotFoundError(f"{env_var} does not exist: {candidate}")
            return candidate

    bundled = package_resource_path(*resource_parts)
    if not bundled.is_file():
        raise FileNotFoundError(f"Bundled {label} URDF is missing: {bundled}")
    return bundled


def get_nero_urdf_path(explicit_path: str | os.PathLike[str] | None = None) -> Path:
    """Resolve the Nero URDF path.

    Priority:
    1. explicit ``urdf_path``
    2. ``NERO_URDF_PATH``
    3. bundled ``assets/nero/nero_description.urdf``
    """
    return resolve_urdf_path(
        explicit_path,
        env_var="NERO_URDF_PATH",
        resource_parts=("assets", "nero", "nero_description.urdf"),
        label="Nero",
    )
