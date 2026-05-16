"""Legacy compatibility imports for older Nero kinematics code.

New code should import from ``pinocchio_kinematics_lite`` directly.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np

_SRC_ROOT = Path(__file__).resolve().parents[2] / "src"
if _SRC_ROOT.is_dir() and str(_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(_SRC_ROOT))

from pinocchio_kinematics_lite import NeroKinematics
from pinocchio_kinematics_lite.transforms import pose_matrix_from_xyz_rpy, xyz_rpy_from_pose_matrix


@dataclass
class ContinuityRuntimeState:
    q_prev: np.ndarray
    q_prev2: np.ndarray | None = None
    theta0_prev: float | None = None
    q_lock: np.ndarray | None = None


class Pinocchio_Solver:
    """Compatibility wrapper around ``pinocchio_kinematics_lite.NeroKinematics``."""

    def __init__(
        self,
        joint_limits=None,
        dt: float = 0.05,
        n_psi: int = 61,
        urdf_path=None,
        ee_frame_name: str = "link7",
        tcp_offset=None,
        max_iterations: int = 100,
        damping: float = 1e-4,
        tol_pos: float = 1e-4,
        tol_rot: float = 1e-3,
    ):
        warnings.warn(
            "nero.kinematics.Pinocchio_Solver is deprecated; use "
            "pinocchio_kinematics_lite.NeroKinematics instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        if tcp_offset is not None and np.linalg.norm(np.asarray(tcp_offset, dtype=float)) > 0.0:
            raise NotImplementedError(
                "The compatibility wrapper does not apply TCP offsets. "
                "Use PinocchioKinematics with a URDF frame that represents your TCP."
            )
        self.kinematics = NeroKinematics(urdf_path=urdf_path, end_effector_frame=ee_frame_name)
        self.joint_limits = (
            np.asarray(joint_limits, dtype=float)
            if joint_limits is not None
            else self.kinematics.get_joint_limits()
        )
        self.dt = dt
        self.n_psi = n_psi
        self.max_iterations = int(max_iterations)
        self.damping = float(damping)
        self.tol_pos = float(tol_pos)
        self.tol_rot = float(tol_rot)
        self.urdf_path = self.kinematics.urdf_path
        self.ee_frame_name = self.kinematics.end_effector_frame
        self.joint_names = self.kinematics.list_joints()
        self.frame_names = self.kinematics.list_frames()
        self.last_report = None
        self.state = None

    def init_state(self, current_q):
        q = self._clamp_joints(current_q)
        self.state = ContinuityRuntimeState(q_prev=q)

    def solve(self, target_pose, limit_output_step: bool = True):
        del limit_output_step
        target = pose_matrix_from_xyz_rpy(target_pose[:3], target_pose[3:])
        q_init = self.state.q_prev if self.state is not None else np.zeros(len(self.joint_names))
        result = self.kinematics.inverse_kinematics(
            target,
            q_init=q_init,
            max_iters=self.max_iterations,
            pos_tol=self.tol_pos,
            ori_tol=self.tol_rot,
            damping=self.damping,
        )
        self.last_report = result.as_dict()
        if not result.success or result.q is None:
            return None
        self.state = ContinuityRuntimeState(q_prev=result.q.copy())
        return result.q

    def fk_matrix(self, q):
        return self.kinematics.forward_kinematics(q)

    def fk_pose(self, q):
        xyz, rpy = xyz_rpy_from_pose_matrix(self.fk_matrix(q))
        return np.concatenate([xyz, rpy])

    def jacobian_matrix(self, q, reference_frame=None):
        del reference_frame
        return self.kinematics.jacobian(q)

    jacobian = jacobian_matrix

    def _clamp_joints(self, q):
        return self.kinematics.clip_to_joint_limits(q)


__all__ = ["ContinuityRuntimeState", "Pinocchio_Solver"]
