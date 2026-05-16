"""Small Jacobian convenience helpers."""

from __future__ import annotations

import numpy as np

from .kinematics import PinocchioKinematics


def frame_jacobian(
    kin: PinocchioKinematics,
    q: np.ndarray,
    frame_name: str | None = None,
) -> np.ndarray:
    return kin.jacobian(q, frame_name=frame_name)
