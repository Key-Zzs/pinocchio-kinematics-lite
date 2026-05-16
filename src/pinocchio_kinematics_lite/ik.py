"""Numerical inverse kinematics result types."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class IKResult:
    """Structured result returned by ``PinocchioKinematics.inverse_kinematics``."""

    success: bool
    q: Optional[np.ndarray]
    position_error: Optional[float]
    orientation_error: Optional[float]
    iterations: int
    solve_time_ms: float
    reason: str
    best_q: Optional[np.ndarray]
    last_q: Optional[np.ndarray]

    def as_dict(self) -> dict:
        return {
            "success": bool(self.success),
            "q": None if self.q is None else np.asarray(self.q, dtype=float).tolist(),
            "position_error": self.position_error,
            "orientation_error": self.orientation_error,
            "iterations": int(self.iterations),
            "solve_time_ms": float(self.solve_time_ms),
            "reason": self.reason,
            "best_q": None if self.best_q is None else np.asarray(self.best_q, dtype=float).tolist(),
            "last_q": None if self.last_q is None else np.asarray(self.last_q, dtype=float).tolist(),
        }
