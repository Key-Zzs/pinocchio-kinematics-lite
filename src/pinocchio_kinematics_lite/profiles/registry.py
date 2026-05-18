"""Built-in robot-profile registry."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from ..kinematics import PinocchioKinematics
from ..resources import package_resource_name, resolve_urdf_path


NERO_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
FRANKA_PANDA_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
ARX_R5_RIGHT_ARM_JOINT_NAMES = tuple(f"right_joint{i}" for i in range(1, 7))
ARX_R5_LEFT_ARM_JOINT_NAMES = tuple(f"left_joint{i}" for i in range(1, 7))

NERO_JOINT_LIMITS = (
    (-2.705261, 2.705261),
    (-1.745330, 1.745330),
    (-2.757621, 2.757621),
    (-1.012291, 2.146755),
    (-2.757621, 2.757621),
    (-0.733039, 0.959932),
    (-1.570797, 1.570797),
)

FRANKA_PANDA_JOINT_LIMITS = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)


@dataclass(frozen=True)
class RobotProfile:
    """Configuration needed to construct kinematics for a bundled robot."""

    name: str
    resource_parts: tuple[str, ...]
    end_effector_frame: str
    active_joint_names: tuple[str, ...]
    root_frame: str | None = None
    joint_limits: tuple[tuple[float, float], ...] | None = None
    env_var: str | None = None
    model_name: str | None = None
    aliases: tuple[str, ...] = ()
    description: str = ""

    @property
    def urdf_resource(self) -> str:
        return package_resource_name(*self.resource_parts)


_PROFILE_DEFINITIONS = (
    RobotProfile(
        name="nero",
        aliases=("nero_7dof",),
        resource_parts=("assets", "nero", "nero_description.urdf"),
        end_effector_frame="link7",
        active_joint_names=NERO_JOINT_NAMES,
        joint_limits=NERO_JOINT_LIMITS,
        env_var="NERO_URDF_PATH",
        model_name="nero_7dof",
        description="Nero 7-DoF arm",
    ),
    RobotProfile(
        name="franka_panda",
        aliases=("franka-panda", "panda"),
        resource_parts=("assets", "franka_panda", "franka_panda.urdf"),
        end_effector_frame="link7",
        active_joint_names=FRANKA_PANDA_JOINT_NAMES,
        joint_limits=FRANKA_PANDA_JOINT_LIMITS,
        env_var="FRANKA_PANDA_URDF_PATH",
        model_name="franka_panda",
        description="Franka Panda arm without gripper actuation",
    ),
    RobotProfile(
        name="franka_panda_robotiq",
        aliases=("franka-panda-robotiq", "panda_robotiq"),
        resource_parts=("assets", "franka_panda", "franka_panda_robotiq.urdf"),
        end_effector_frame="robotiq_arg2f_base_link",
        active_joint_names=FRANKA_PANDA_JOINT_NAMES,
        joint_limits=FRANKA_PANDA_JOINT_LIMITS,
        env_var="FRANKA_PANDA_ROBOTIQ_URDF_PATH",
        model_name="franka_panda_robotiq",
        description="Franka Panda arm with a Robotiq 2F gripper mounted at the wrist",
    ),
    RobotProfile(
        name="arx_r5",
        aliases=("arx-r5", "arx_r5_right", "arx-r5-right"),
        resource_parts=("assets", "arx_r5", "dual_R5a.urdf"),
        end_effector_frame="right_link6",
        active_joint_names=ARX_R5_RIGHT_ARM_JOINT_NAMES,
        root_frame="right_base_link",
        env_var="ARX_R5_URDF_PATH",
        model_name="arx_r5_right",
        description="ARX R5 right arm from the bundled dual-arm URDF",
    ),
    RobotProfile(
        name="arx_r5_left",
        aliases=("arx-r5-left",),
        resource_parts=("assets", "arx_r5", "dual_R5a.urdf"),
        end_effector_frame="left_link6",
        active_joint_names=ARX_R5_LEFT_ARM_JOINT_NAMES,
        root_frame="left_base_link",
        env_var="ARX_R5_URDF_PATH",
        model_name="arx_r5_left",
        description="ARX R5 left arm from the bundled dual-arm URDF",
    ),
)


def _normalize_profile_name(name: str) -> str:
    return name.strip().lower().replace("-", "_")


_PROFILES_BY_NAME = {profile.name: profile for profile in _PROFILE_DEFINITIONS}
_PROFILES_BY_ALIAS = {
    _normalize_profile_name(alias): profile
    for profile in _PROFILE_DEFINITIONS
    for alias in profile.aliases
}


def list_robot_profiles(*, include_aliases: bool = False) -> tuple[str, ...]:
    """Return supported built-in robot-profile names."""
    names = [profile.name for profile in _PROFILE_DEFINITIONS]
    if include_aliases:
        names.extend(alias for profile in _PROFILE_DEFINITIONS for alias in profile.aliases)
    return tuple(dict.fromkeys(names))


def get_robot_profile(profile_name: str) -> RobotProfile:
    """Return a built-in robot profile by canonical name or alias."""
    key = _normalize_profile_name(profile_name)
    profile = _PROFILES_BY_NAME.get(key) or _PROFILES_BY_ALIAS.get(key)
    if profile is None:
        supported = ", ".join(list_robot_profiles(include_aliases=True))
        raise ValueError(f"Unsupported robot profile {profile_name!r}. Supported profiles: {supported}")
    return profile


def get_robot_urdf_path(
    profile_name: str,
    explicit_path: str | os.PathLike[str] | None = None,
) -> Path:
    """Resolve a built-in robot profile URDF path."""
    profile = get_robot_profile(profile_name)
    return resolve_urdf_path(
        explicit_path,
        env_var=profile.env_var,
        resource_parts=profile.resource_parts,
        label=profile.name,
    )


def create_robot_kinematics(
    profile_name: str,
    *,
    urdf_path: str | Path | None = None,
    end_effector_frame: str | None = None,
    root_frame: str | None = None,
    active_joint_names: Iterable[str] | None = None,
    locked_joint_names: Iterable[str] | None = None,
    joint_limits: Iterable[Iterable[float]] | None = None,
    mesh_dir: str | Path | None = None,
    model_name: str | None = None,
) -> PinocchioKinematics:
    """Construct ``PinocchioKinematics`` from a built-in robot profile."""
    profile = get_robot_profile(profile_name)
    uses_bundled_urdf = urdf_path is None and not (profile.env_var and os.getenv(profile.env_var))
    resolved_urdf = get_robot_urdf_path(profile.name, urdf_path)
    effective_joint_limits = profile.joint_limits if active_joint_names is None and joint_limits is None else joint_limits

    kin = PinocchioKinematics(
        urdf_path=resolved_urdf,
        end_effector_frame=end_effector_frame or profile.end_effector_frame,
        root_frame=profile.root_frame if root_frame is None else root_frame,
        active_joint_names=profile.active_joint_names if active_joint_names is None else active_joint_names,
        locked_joint_names=locked_joint_names,
        joint_limits=effective_joint_limits,
        mesh_dir=mesh_dir,
        model_name=profile.model_name if model_name is None else model_name,
    )
    kin.robot_profile = profile.name
    kin.filesystem_urdf_path = kin.resolved_urdf_path
    if uses_bundled_urdf:
        kin.urdf_path = profile.urdf_resource
        kin.resolved_urdf_path = profile.urdf_resource
    return kin
