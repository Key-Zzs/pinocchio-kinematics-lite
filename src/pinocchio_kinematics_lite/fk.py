"""Small forward-kinematics convenience helpers."""

from __future__ import annotations

import numpy as np

from .kinematics import PinocchioKinematics


def forward_kinematics(
    kin: PinocchioKinematics,
    q: np.ndarray,
    frame_name: str | None = None,
) -> np.ndarray:
    return kin.forward_kinematics(q, frame_name=frame_name)
