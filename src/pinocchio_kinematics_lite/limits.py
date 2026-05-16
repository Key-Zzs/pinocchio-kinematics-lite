"""Joint limit helpers for active-joint vectors."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class JointLimitViolation:
    """One joint-limit violation for a single active joint."""

    index: int
    joint_name: str
    value: float
    lower: float
    upper: float


def as_joint_limits(joint_limits: Iterable[Iterable[float]]) -> np.ndarray:
    limits = np.asarray(joint_limits, dtype=float)
    if limits.ndim != 2 or limits.shape[1] != 2:
        raise ValueError(f"Expected joint limits with shape (N, 2), got {limits.shape}")
    return limits


def clip_to_joint_limits(q: np.ndarray, joint_limits: np.ndarray) -> np.ndarray:
    limits = as_joint_limits(joint_limits)
    q_arr = np.asarray(q, dtype=float).reshape(-1)
    if q_arr.size != limits.shape[0]:
        raise ValueError(f"Expected q with {limits.shape[0]} values, got {q_arr.size}")
    lower = np.where(np.isfinite(limits[:, 0]), limits[:, 0], q_arr)
    upper = np.where(np.isfinite(limits[:, 1]), limits[:, 1], q_arr)
    return np.minimum(np.maximum(q_arr, lower), upper)


def joint_limit_violations(
    q: np.ndarray,
    joint_limits: np.ndarray,
    joint_names: Iterable[str] | None = None,
    *,
    tol: float = 1e-9,
) -> list[JointLimitViolation]:
    limits = as_joint_limits(joint_limits)
    q_arr = np.asarray(q, dtype=float).reshape(-1)
    if q_arr.size != limits.shape[0]:
        raise ValueError(f"Expected q with {limits.shape[0]} values, got {q_arr.size}")
    names = list(joint_names or [f"joint_{i}" for i in range(q_arr.size)])
    if len(names) != q_arr.size:
        raise ValueError("joint_names length must match q length")

    violations: list[JointLimitViolation] = []
    for i, value in enumerate(q_arr):
        lower = float(limits[i, 0])
        upper = float(limits[i, 1])
        if np.isfinite(lower) and value < lower - tol:
            violations.append(JointLimitViolation(i, names[i], float(value), lower, upper))
        elif np.isfinite(upper) and value > upper + tol:
            violations.append(JointLimitViolation(i, names[i], float(value), lower, upper))
    return violations


def within_joint_limits(q: np.ndarray, joint_limits: np.ndarray, *, tol: float = 1e-9) -> bool:
    return not joint_limit_violations(q, joint_limits, tol=tol)


def sample_random_q(
    rng: np.random.Generator,
    joint_limits: np.ndarray,
    *,
    margin: float | np.ndarray = 0.0,
    unbounded_limit: float = np.pi,
) -> np.ndarray:
    """Uniformly sample one active-joint vector inside finite limits.

    Continuous or otherwise unbounded joints are sampled from
    ``[-unbounded_limit, unbounded_limit]``.
    """
    limits = as_joint_limits(joint_limits)
    margin_arr = np.broadcast_to(np.asarray(margin, dtype=float), limits[:, 0].shape)
    lower = limits[:, 0].copy()
    upper = limits[:, 1].copy()
    lower[~np.isfinite(lower)] = -float(unbounded_limit)
    upper[~np.isfinite(upper)] = float(unbounded_limit)
    lower = lower + margin_arr
    upper = upper - margin_arr
    if np.any(lower > upper):
        raise ValueError("Joint-limit margin is larger than at least one joint range")
    return rng.uniform(lower, upper)
