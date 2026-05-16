"""Built-in robot profiles."""

from .nero import (
    DEFAULT_NERO_END_EFFECTOR_FRAME,
    DEFAULT_NERO_JOINT_LIMITS,
    DEFAULT_NERO_JOINT_NAMES,
    NeroKinematics,
)

__all__ = [
    "DEFAULT_NERO_END_EFFECTOR_FRAME",
    "DEFAULT_NERO_JOINT_LIMITS",
    "DEFAULT_NERO_JOINT_NAMES",
    "NeroKinematics",
]
