"""Generic Pinocchio-backed URDF kinematics."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Iterable

import numpy as np

from .ik import IKResult
from .limits import (
    JointLimitViolation,
    clip_to_joint_limits,
    joint_limit_violations,
    sample_random_q as sample_from_limits,
)
from .transforms import as_pose_matrix, matrix_to_rpy, pose_errors

try:  # pragma: no cover - exercised only when Pinocchio is not installed.
    import pinocchio as pin
except ImportError:  # pragma: no cover
    pin = None


def _require_pinocchio():
    if pin is None:
        raise RuntimeError(
            "Pinocchio is required for kinematics. Install it with "
            "conda install -c conda-forge pinocchio eigenpy -y, or install a "
            "platform-compatible pip package if one is available for your system."
        )
    return pin


class PinocchioKinematics:
    """Robot-agnostic FK, IK, Jacobian, and joint-limit helper for URDF arms."""

    def __init__(
        self,
        urdf_path: str | Path,
        end_effector_frame: str,
        root_frame: str | None = None,
        active_joint_names: Iterable[str] | None = None,
        locked_joint_names: Iterable[str] | None = None,
        mesh_dir: str | Path | None = None,
        model_name: str | None = None,
    ) -> None:
        pin_mod = _require_pinocchio()
        self.resolved_urdf_path = str(Path(urdf_path).expanduser().resolve())
        self.urdf_path = self.resolved_urdf_path
        if not Path(self.resolved_urdf_path).is_file():
            raise FileNotFoundError(f"URDF does not exist: {self.resolved_urdf_path}")
        self.end_effector_frame = str(end_effector_frame)
        self.root_frame = None if root_frame is None else str(root_frame)
        self.mesh_dir = None if mesh_dir is None else str(Path(mesh_dir).expanduser().resolve())
        self.model_name = model_name

        self.model = pin_mod.buildModelFromUrdf(self.resolved_urdf_path)
        self.data = self.model.createData()

        self._locked_joint_names = set(locked_joint_names or [])
        self.active_joint_names = self._resolve_active_joint_names(active_joint_names)
        self._active_joint_ids = [self._joint_id(name) for name in self.active_joint_names]
        self._active_q_idx = [int(self.model.joints[joint_id].idx_q) for joint_id in self._active_joint_ids]
        self._active_v_idx = [int(self.model.joints[joint_id].idx_v) for joint_id in self._active_joint_ids]
        self.nq_active = len(self.active_joint_names)

        self._joint_limits = self._resolve_joint_limits()
        self._default_frame_id = self._frame_id(self.end_effector_frame)
        self._root_frame_id = None if self.root_frame is None else self._frame_id(self.root_frame)

    @property
    def joint_names(self) -> list[str]:
        return list(self.active_joint_names)

    @property
    def frame_names(self) -> list[str]:
        return self.list_frames()

    def list_frames(self) -> list[str]:
        return [frame.name for frame in self.model.frames]

    def list_joints(self) -> list[str]:
        return list(self.active_joint_names)

    def get_joint_limits(self) -> np.ndarray:
        return self._joint_limits.copy()

    def check_joint_limits(self, q: np.ndarray, *, tol: float = 1e-9) -> bool:
        self._validate_q(q)
        return not self.joint_limit_violations(q, tol=tol)

    def joint_limit_violations(
        self,
        q: np.ndarray,
        *,
        tol: float = 1e-9,
    ) -> list[JointLimitViolation]:
        self._validate_q(q)
        return joint_limit_violations(q, self._joint_limits, self.active_joint_names, tol=tol)

    def clip_to_joint_limits(self, q: np.ndarray) -> np.ndarray:
        self._validate_q(q)
        return clip_to_joint_limits(q, self._joint_limits)

    def sample_random_q(self, seed: int | None = None) -> np.ndarray:
        rng = np.random.default_rng(seed)
        return sample_from_limits(rng, self._joint_limits)

    def forward_kinematics(self, q: np.ndarray, frame_name: str | None = None) -> np.ndarray:
        q_full = self._to_full_q(q)
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        frame_id = self._frame_id(frame_name or self.end_effector_frame)
        T_world_frame = self._placement_to_matrix(self.data.oMf[frame_id])
        if self._root_frame_id is None:
            return T_world_frame
        T_world_root = self._placement_to_matrix(self.data.oMf[self._root_frame_id])
        return np.linalg.inv(T_world_root) @ T_world_frame

    fk_matrix = forward_kinematics

    def fk_pose(self, q: np.ndarray, frame_name: str | None = None) -> np.ndarray:
        T = self.forward_kinematics(q, frame_name=frame_name)
        return np.concatenate([T[:3, 3], matrix_to_rpy(T[:3, :3])])

    def jacobian(self, q: np.ndarray, frame_name: str | None = None) -> np.ndarray:
        q_full = self._to_full_q(q)
        frame_id = self._frame_id(frame_name or self.end_effector_frame)
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        J_full = pin.computeFrameJacobian(
            self.model,
            self.data,
            q_full,
            frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        J = np.asarray(J_full[:, self._active_v_idx], dtype=float).copy()
        if self._root_frame_id is not None:
            R_root_world = self.data.oMf[self._root_frame_id].rotation.T
            J[:3, :] = R_root_world @ J[:3, :]
            J[3:, :] = R_root_world @ J[3:, :]
        return J

    jacobian_matrix = jacobian

    def inverse_kinematics(
        self,
        target_pose,
        q_init: np.ndarray,
        frame_name: str | None = None,
        max_iters: int = 100,
        pos_tol: float = 1e-4,
        ori_tol: float = 1e-3,
        damping: float = 1e-4,
        max_step: float = 0.2,
    ) -> IKResult:
        """Solve numerical IK with damped least squares.

        ``target_pose`` is interpreted in the same frame as
        ``forward_kinematics`` returns. With the default ``root_frame=None`` this
        is the URDF world/model frame. The implementation focuses on fixed-base
        serial manipulators with one scalar coordinate per active joint.
        """
        start = time.perf_counter()
        frame_id = self._frame_id(frame_name or self.end_effector_frame)
        target_user = as_pose_matrix(target_pose)
        q = self.clip_to_joint_limits(q_init)
        q_full = self._to_full_q(q)
        target_world = self._target_to_world(target_user, q_full)
        target_se3 = pin.SE3(target_world[:3, :3], target_world[:3, 3])

        best_q = q.copy()
        last_q = q.copy()
        best_pos_err: float | None = None
        best_ori_err: float | None = None
        best_score = float("inf")
        final_pos_err: float | None = None
        final_ori_err: float | None = None
        reason = "max_iterations"
        iterations = 0

        for iteration in range(1, int(max_iters) + 1):
            iterations = iteration
            pin.forwardKinematics(self.model, self.data, q_full)
            pin.updateFramePlacements(self.model, self.data)
            current_se3 = self.data.oMf[frame_id]
            current_world = self._placement_to_matrix(current_se3)
            final_pos_err, final_ori_err = pose_errors(current_world, target_world)
            score = final_pos_err + final_ori_err
            if score < best_score:
                best_score = score
                best_q = q.copy()
                best_pos_err = final_pos_err
                best_ori_err = final_ori_err
            if final_pos_err <= pos_tol and final_ori_err <= ori_tol:
                reason = "converged"
                solve_time_ms = (time.perf_counter() - start) * 1000.0
                return IKResult(
                    success=True,
                    q=q.copy(),
                    position_error=final_pos_err,
                    orientation_error=final_ori_err,
                    iterations=iterations,
                    solve_time_ms=solve_time_ms,
                    reason=reason,
                    best_q=best_q.copy(),
                    last_q=q.copy(),
                )

            err6 = pin.log6(current_se3.inverse() * target_se3).vector
            J_full = pin.computeFrameJacobian(
                self.model,
                self.data,
                q_full,
                frame_id,
                pin.ReferenceFrame.LOCAL,
            )
            J = np.asarray(J_full[:, self._active_v_idx], dtype=float)
            H = J @ J.T + float(damping) * np.eye(6)
            try:
                dq = J.T @ np.linalg.solve(H, err6)
            except np.linalg.LinAlgError:
                reason = "singular_linear_system"
                break
            dq = np.asarray(dq, dtype=float).reshape(-1)
            if not np.all(np.isfinite(dq)):
                reason = "non_finite_step"
                break
            if max_step is not None and max_step > 0.0:
                dq = np.clip(dq, -float(max_step), float(max_step))
            q = self.clip_to_joint_limits(q + dq)
            last_q = q.copy()
            q_full = self._to_full_q(q)

        solve_time_ms = (time.perf_counter() - start) * 1000.0
        return IKResult(
            success=False,
            q=None,
            position_error=best_pos_err,
            orientation_error=best_ori_err,
            iterations=iterations,
            solve_time_ms=solve_time_ms,
            reason=reason,
            best_q=None if best_q is None else best_q.copy(),
            last_q=last_q.copy(),
        )

    def _resolve_active_joint_names(self, active_joint_names: Iterable[str] | None) -> list[str]:
        if active_joint_names is not None:
            names = [str(name) for name in active_joint_names]
            overlap = self._locked_joint_names.intersection(names)
            if overlap:
                raise ValueError(f"Active joints cannot also be locked: {sorted(overlap)}")
            for name in names:
                joint_id = self._joint_id(name)
                joint = self.model.joints[joint_id]
                if joint.nq != 1 or joint.nv != 1:
                    raise NotImplementedError(
                        f"Active joint {name!r} has nq={joint.nq}, nv={joint.nv}; "
                        "only scalar revolute/prismatic joints are currently supported."
                    )
            return names

        names: list[str] = []
        for joint_id in range(1, self.model.njoints):
            name = self.model.names[joint_id]
            if name in self._locked_joint_names:
                continue
            joint = self.model.joints[joint_id]
            if joint.nq == 1 and joint.nv == 1:
                names.append(name)
        if not names:
            raise ValueError("No scalar active joints were found in the URDF")
        return names

    def _resolve_joint_limits(self) -> np.ndarray:
        lower = np.asarray(self.model.lowerPositionLimit, dtype=float)
        upper = np.asarray(self.model.upperPositionLimit, dtype=float)
        limits = np.zeros((self.nq_active, 2), dtype=float)
        for i, q_idx in enumerate(self._active_q_idx):
            limits[i, 0] = lower[q_idx]
            limits[i, 1] = upper[q_idx]
        return limits

    def _joint_id(self, joint_name: str) -> int:
        joint_id = int(self.model.getJointId(joint_name))
        if joint_id >= self.model.njoints or self.model.names[joint_id] != joint_name:
            raise ValueError(f"Joint {joint_name!r} not found in URDF model")
        return joint_id

    def _frame_id(self, frame_name: str) -> int:
        frame_id = int(self.model.getFrameId(frame_name))
        if frame_id >= self.model.nframes or self.model.frames[frame_id].name != frame_name:
            raise ValueError(f"Frame {frame_name!r} not found in URDF model")
        return frame_id

    def _validate_q(self, q: np.ndarray) -> np.ndarray:
        q_arr = np.asarray(q, dtype=float).reshape(-1)
        if q_arr.size != self.nq_active:
            raise ValueError(f"Expected q with {self.nq_active} values, got {q_arr.size}")
        return q_arr

    def _to_full_q(self, q: np.ndarray) -> np.ndarray:
        q_active = self._validate_q(q)
        q_full = pin.neutral(self.model)
        for value, q_idx in zip(q_active, self._active_q_idx):
            q_full[q_idx] = float(value)
        return q_full

    def _target_to_world(self, target_user: np.ndarray, q_full: np.ndarray) -> np.ndarray:
        if self._root_frame_id is None:
            return target_user
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        T_world_root = self._placement_to_matrix(self.data.oMf[self._root_frame_id])
        return T_world_root @ target_user

    @staticmethod
    def _placement_to_matrix(placement) -> np.ndarray:
        T = np.eye(4, dtype=float)
        T[:3, :3] = np.asarray(placement.rotation, dtype=float)
        T[:3, 3] = np.asarray(placement.translation, dtype=float)
        return T
