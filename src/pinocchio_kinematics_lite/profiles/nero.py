"""Built-in Nero 7-DoF profile."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from ..kinematics import PinocchioKinematics
from .registry import (
    NERO_JOINT_LIMITS,
    NERO_JOINT_NAMES,
    get_robot_profile,
    get_robot_urdf_path,
)


_NERO_PROFILE = get_robot_profile("nero")

DEFAULT_NERO_JOINT_NAMES = list(NERO_JOINT_NAMES)
DEFAULT_NERO_END_EFFECTOR_FRAME = _NERO_PROFILE.end_effector_frame
DEFAULT_NERO_URDF_RESOURCE = _NERO_PROFILE.urdf_resource
DEFAULT_NERO_JOINT_LIMITS = np.array(NERO_JOINT_LIMITS, dtype=float)


class NeroKinematics(PinocchioKinematics):
    """Thin Nero profile over the generic ``PinocchioKinematics`` class."""

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        end_effector_frame: str = DEFAULT_NERO_END_EFFECTOR_FRAME,
        root_frame: str | None = None,
        active_joint_names: list[str] | None = None,
        locked_joint_names: list[str] | None = None,
        joint_limits: list[list[float]] | np.ndarray | None = None,
        mesh_dir: str | Path | None = None,
        model_name: str | None = "nero_7dof",
    ) -> None:
        uses_bundled_urdf = urdf_path is None and not os.getenv("NERO_URDF_PATH")
        resolved_urdf = get_robot_urdf_path("nero", urdf_path)
        effective_joint_limits = (
            DEFAULT_NERO_JOINT_LIMITS if active_joint_names is None and joint_limits is None else joint_limits
        )
        super().__init__(
            urdf_path=resolved_urdf,
            end_effector_frame=end_effector_frame,
            root_frame=root_frame,
            active_joint_names=DEFAULT_NERO_JOINT_NAMES if active_joint_names is None else active_joint_names,
            locked_joint_names=locked_joint_names,
            joint_limits=effective_joint_limits,
            mesh_dir=mesh_dir,
            model_name=model_name,
        )
        self.robot_profile = "nero"
        self.filesystem_urdf_path = self.resolved_urdf_path
        if uses_bundled_urdf:
            self.urdf_path = DEFAULT_NERO_URDF_RESOURCE
            self.resolved_urdf_path = DEFAULT_NERO_URDF_RESOURCE
