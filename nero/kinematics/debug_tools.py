"""Debug helpers for Nero kinematics tests and benchmarks.

The helpers in this module intentionally depend only on NumPy. They operate on
plain joint vectors and homogeneous transforms, so they can be reused with
Pinocchio, analytic solvers, or other generic robot backends.
"""

from __future__ import annotations

import math
from typing import Callable, Iterable, Tuple

import numpy as np


DEFAULT_NERO_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
DEFAULT_NERO_EE_FRAME = "link7"
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


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation matrix for roll-pitch-yaw using ZYX convention."""
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
    """Inverse of rpy_to_matrix for ZYX convention."""
    R = np.asarray(rotation, dtype=float)
    pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
    cp = math.cos(pitch)
    if abs(cp) < 1e-9:
        roll = 0.0
        yaw = math.atan2(float(-R[0, 1]), float(R[1, 1]))
    else:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    return np.array([roll, pitch, yaw], dtype=float)


def pose6_to_matrix(pose: Iterable[float]) -> np.ndarray:
    """Convert [x, y, z, roll, pitch, yaw] to a 4x4 transform."""
    pose = np.asarray(pose, dtype=float).reshape(6)
    T = np.eye(4, dtype=float)
    T[:3, :3] = rpy_to_matrix(pose[3], pose[4], pose[5])
    T[:3, 3] = pose[:3]
    return T


def matrix_to_pose6(T: np.ndarray) -> np.ndarray:
    """Convert a 4x4 transform to [x, y, z, roll, pitch, yaw]."""
    T = np.asarray(T, dtype=float)
    return np.concatenate([T[:3, 3], matrix_to_rpy(T[:3, :3])])


def rotation_angle_error(actual_R: np.ndarray, target_R: np.ndarray) -> float:
    """Geodesic SO(3) angle error in radians."""
    R_delta = np.asarray(actual_R, dtype=float).T @ np.asarray(target_R, dtype=float)
    cos_angle = (float(np.trace(R_delta)) - 1.0) * 0.5
    return float(math.acos(max(-1.0, min(1.0, cos_angle))))


def pose_errors(actual_T: np.ndarray, target_T: np.ndarray) -> Tuple[float, float]:
    """Return (position_error_m, orientation_error_rad)."""
    actual_T = np.asarray(actual_T, dtype=float)
    target_T = np.asarray(target_T, dtype=float)
    pos_err = float(np.linalg.norm(actual_T[:3, 3] - target_T[:3, 3]))
    ori_err = rotation_angle_error(actual_T[:3, :3], target_T[:3, :3])
    return pos_err, ori_err


def sample_random_q(
    rng: np.random.Generator,
    joint_limits: np.ndarray,
    num_samples: int = 1,
    margin: float | np.ndarray = 0.0,
) -> np.ndarray:
    """Uniformly sample joint vectors inside joint limits."""
    limits = np.asarray(joint_limits, dtype=float)
    margin_arr = np.broadcast_to(np.asarray(margin, dtype=float), limits[:, 0].shape)
    low = limits[:, 0] + margin_arr
    high = limits[:, 1] - margin_arr
    if np.any(low > high):
        raise ValueError("Joint-limit margin is larger than at least one joint range")
    return rng.uniform(low, high, size=(int(num_samples), limits.shape[0]))


def clip_to_joint_limits(q: np.ndarray, joint_limits: np.ndarray) -> np.ndarray:
    limits = np.asarray(joint_limits, dtype=float)
    return np.clip(np.asarray(q, dtype=float), limits[:, 0], limits[:, 1])


def joint_limit_violation(
    q: np.ndarray | None,
    joint_limits: np.ndarray,
    tol: float = 1e-9,
) -> bool:
    if q is None:
        return False
    q = np.asarray(q, dtype=float).reshape(-1)
    limits = np.asarray(joint_limits, dtype=float)
    return bool(np.any(q < limits[:, 0] - tol) or np.any(q > limits[:, 1] + tol))


def rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
    """Return the rotation vector for a small or finite SO(3) matrix."""
    R = np.asarray(rotation, dtype=float)
    angle = rotation_angle_error(np.eye(3), R)
    vee = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    )
    if angle < 1e-10:
        return 0.5 * vee
    sin_angle = math.sin(angle)
    if abs(sin_angle) < 1e-10:
        return 0.5 * vee
    return (angle / (2.0 * sin_angle)) * vee


def numerical_jacobian(
    fk_matrix_fn: Callable[[np.ndarray], np.ndarray],
    q: np.ndarray,
    eps: float = 1e-6,
) -> np.ndarray:
    """
    Central finite-difference Jacobian for a 4x4 FK function.

    Rows are ordered as [linear_velocity; angular_velocity]. Angular velocity
    is expressed in the world frame via log(R_plus * R_minus.T) / (2 eps).
    """
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


def scalar_stats(values: Iterable[float]) -> dict[str, float]:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return {
            "mean": float("nan"),
            "median": float("nan"),
            "p90": float("nan"),
            "p95": float("nan"),
            "p99": float("nan"),
            "max": float("nan"),
        }
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "max": float(np.max(arr)),
    }
