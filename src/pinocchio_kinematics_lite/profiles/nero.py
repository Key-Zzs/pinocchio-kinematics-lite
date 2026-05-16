"""Built-in AgileX Nero 7-DoF profile."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..kinematics import PinocchioKinematics
from ..resources import get_nero_urdf_path


DEFAULT_NERO_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
DEFAULT_NERO_END_EFFECTOR_FRAME = "link7"
DEFAULT_NERO_JOINT_LIMITS = np.array(
    [
        [-2.705261, 2.705261],
        [-1.745330, 1.745330],
        [-2.757621, 2.757621],
        [-1.012291, 2.146755],
        [-2.757621, 2.757621],
        [-0.733039, 0.959932],
        [-1.570797, 1.570797],
    ],
    dtype=float,
)


class NeroKinematics(PinocchioKinematics):
    """Thin Nero profile over the generic ``PinocchioKinematics`` class."""

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        end_effector_frame: str = DEFAULT_NERO_END_EFFECTOR_FRAME,
        active_joint_names: list[str] | None = None,
        locked_joint_names: list[str] | None = None,
        mesh_dir: str | Path | None = None,
        model_name: str | None = "nero_7dof",
    ) -> None:
        resolved_urdf = get_nero_urdf_path(urdf_path)
        super().__init__(
            urdf_path=resolved_urdf,
            end_effector_frame=end_effector_frame,
            active_joint_names=DEFAULT_NERO_JOINT_NAMES if active_joint_names is None else active_joint_names,
            locked_joint_names=locked_joint_names,
            mesh_dir=mesh_dir,
            model_name=model_name,
        )
