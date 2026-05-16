"""Pose and transform helpers used by the generic kinematics API."""

from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np


def _as_xyz(values: Iterable[float]) -> np.ndarray:
    xyz = np.asarray(values, dtype=float).reshape(-1)
    if xyz.size != 3:
        raise ValueError(f"Expected 3 position values, got {xyz.size}")
    return xyz


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a rotation matrix for ZYX roll-pitch-yaw angles."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    """Return ZYX roll-pitch-yaw angles from a rotation matrix."""
    R = np.asarray(rotation, dtype=float)
    if R.shape != (3, 3):
        raise ValueError(f"Expected a 3x3 rotation matrix, got {R.shape}")
    pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
    cp = math.cos(pitch)
    if abs(cp) < 1e-9:
        roll = 0.0
        yaw = math.atan2(float(-R[0, 1]), float(R[1, 1]))
    else:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return np.array([roll, pitch, yaw], dtype=float)


def pose_matrix_from_xyz_rpy(
    xyz: Iterable[float],
    rpy: Iterable[float],
) -> np.ndarray:
    """Build a 4x4 pose matrix from position and ZYX roll-pitch-yaw."""
    angles = np.asarray(rpy, dtype=float).reshape(-1)
    if angles.size != 3:
        raise ValueError(f"Expected 3 RPY values, got {angles.size}")
    T = np.eye(4, dtype=float)
    T[:3, :3] = rpy_matrix(float(angles[0]), float(angles[1]), float(angles[2]))
    T[:3, 3] = _as_xyz(xyz)
    return T


def pose_matrix_from_xyz_quat(
    xyz: Iterable[float],
    quat: Iterable[float],
    *,
    order: str = "xyzw",
) -> np.ndarray:
    """Build a 4x4 pose matrix from position and a quaternion.

    The default quaternion order is ``xyzw``. Pass ``order="wxyz"`` for
    scalar-first inputs.
    """
    q = np.asarray(quat, dtype=float).reshape(-1)
    if q.size != 4:
        raise ValueError(f"Expected 4 quaternion values, got {q.size}")
    if order == "xyzw":
        x, y, z, w = q
    elif order == "wxyz":
        w, x, y, z = q
    else:
        raise ValueError("Quaternion order must be 'xyzw' or 'wxyz'")

    norm = math.sqrt(float(w * w + x * x + y * y + z * z))
    if norm <= 0.0:
        raise ValueError("Quaternion norm must be positive")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    T = np.eye(4, dtype=float)
    T[:3, :3] = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )
    T[:3, 3] = _as_xyz(xyz)
    return T


def xyz_quat_from_pose_matrix(T: np.ndarray, *, order: str = "xyzw") -> tuple[np.ndarray, np.ndarray]:
    """Return ``(xyz, quat)`` from a 4x4 pose matrix."""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 pose matrix, got {T.shape}")
    R = T[:3, :3]
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(R)))
        if idx == 0:
            s = math.sqrt(1.0 + float(R[0, 0] - R[1, 1] - R[2, 2])) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + float(R[1, 1] - R[0, 0] - R[2, 2])) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + float(R[2, 2] - R[0, 0] - R[1, 1])) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    quat_xyzw = np.array([x, y, z, w], dtype=float)
    quat_xyzw /= np.linalg.norm(quat_xyzw)
    if order == "xyzw":
        quat = quat_xyzw
    elif order == "wxyz":
        quat = quat_xyzw[[3, 0, 1, 2]]
    else:
        raise ValueError("Quaternion order must be 'xyzw' or 'wxyz'")
    return T[:3, 3].copy(), quat


def xyz_rpy_from_pose_matrix(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(xyz, rpy)`` from a 4x4 pose matrix."""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 pose matrix, got {T.shape}")
    return T[:3, 3].copy(), matrix_to_rpy(T[:3, :3])


def as_pose_matrix(pose: Any) -> np.ndarray:
    """Convert supported pose formats to a 4x4 homogeneous matrix.

    Supported inputs are a 4x4 matrix, a ``pinocchio.SE3``-like object with
    ``rotation`` and ``translation`` attributes, a dict with ``position`` and
    either ``quaternion`` or ``rpy``, or a tuple ``(position, quaternion_or_rpy)``.
    Tuple orientation values with length 4 are treated as ``xyzw`` quaternions;
    length 3 values are treated as RPY.
    """
    if hasattr(pose, "rotation") and hasattr(pose, "translation"):
        T = np.eye(4, dtype=float)
        T[:3, :3] = np.asarray(pose.rotation, dtype=float)
        T[:3, 3] = np.asarray(pose.translation, dtype=float).reshape(3)
        return T

    if isinstance(pose, dict):
        position = pose.get("position", pose.get("xyz"))
        if position is None:
            raise ValueError("Pose dict must contain 'position' or 'xyz'")
        if "quaternion" in pose:
            return pose_matrix_from_xyz_quat(
                position,
                pose["quaternion"],
                order=str(pose.get("quaternion_order", "xyzw")),
            )
        if "quat" in pose:
            return pose_matrix_from_xyz_quat(
                position,
                pose["quat"],
                order=str(pose.get("quaternion_order", "xyzw")),
            )
        if "rpy" in pose:
            return pose_matrix_from_xyz_rpy(position, pose["rpy"])
        raise ValueError("Pose dict must contain 'quaternion', 'quat', or 'rpy'")

    if isinstance(pose, tuple) and len(pose) == 2:
        position, orientation = pose
        orientation_array = np.asarray(orientation, dtype=float).reshape(-1)
        if orientation_array.size == 4:
            return pose_matrix_from_xyz_quat(position, orientation_array)
        if orientation_array.size == 3:
            return pose_matrix_from_xyz_rpy(position, orientation_array)
        raise ValueError("Tuple orientation must contain 3 RPY or 4 quaternion values")

    arr = np.asarray(pose, dtype=float)
    if arr.shape == (4, 4):
        return arr.copy()
    raise ValueError(
        "Unsupported pose format. Use a 4x4 matrix, pinocchio.SE3, "
        "a pose dict, or (position, orientation)."
    )


def rotation_angle_error(actual_R: np.ndarray, target_R: np.ndarray) -> float:
    """Return the geodesic SO(3) angle error in radians."""
    R_delta = np.asarray(actual_R, dtype=float).T @ np.asarray(target_R, dtype=float)
    cos_angle = (float(np.trace(R_delta)) - 1.0) * 0.5
    return float(math.acos(max(-1.0, min(1.0, cos_angle))))


def pose_errors(actual_T: np.ndarray, target_T: np.ndarray) -> tuple[float, float]:
    """Return ``(position_error_m, orientation_error_rad)`` for two poses."""
    actual = np.asarray(actual_T, dtype=float)
    target = np.asarray(target_T, dtype=float)
    if actual.shape != (4, 4) or target.shape != (4, 4):
        raise ValueError("pose_errors expects two 4x4 matrices")
    pos_err = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
    ori_err = rotation_angle_error(actual[:3, :3], target[:3, :3])
    return pos_err, ori_err


def rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """Return the rotation vector corresponding to a rotation matrix."""
    R = np.asarray(rotation, dtype=float)
    angle = rotation_angle_error(np.eye(3), R)
    vee = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=float,
    )
    if angle < 1e-10:
        return 0.5 * vee
    sin_angle = math.sin(angle)
    if abs(sin_angle) < 1e-10:
        return 0.5 * vee
    return (angle / (2.0 * sin_angle)) * vee


def numerical_jacobian(fk_matrix_fn, q: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Central finite-difference Jacobian for an FK matrix function."""
    q = np.asarray(q, dtype=float).reshape(-1)
    J = np.zeros((6, q.size), dtype=float)
    for idx in range(q.size):
        dq = np.zeros_like(q)
        dq[idx] = eps
        T_plus = np.asarray(fk_matrix_fn(q + dq), dtype=float)
        T_minus = np.asarray(fk_matrix_fn(q - dq), dtype=float)
        J[:3, idx] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2.0 * eps)
        dR = T_plus[:3, :3] @ T_minus[:3, :3].T
        J[3:, idx] = rotation_vector_from_matrix(dR) / (2.0 * eps)
    return J
