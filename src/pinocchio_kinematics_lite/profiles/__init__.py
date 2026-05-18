"""Built-in robot profiles."""

from .nero import (
    DEFAULT_NERO_END_EFFECTOR_FRAME,
    DEFAULT_NERO_JOINT_LIMITS,
    DEFAULT_NERO_JOINT_NAMES,
    NeroKinematics,
)
from .registry import (
    ARX_R5_LEFT_ARM_JOINT_NAMES,
    ARX_R5_RIGHT_ARM_JOINT_NAMES,
    FRANKA_PANDA_JOINT_LIMITS,
    FRANKA_PANDA_JOINT_NAMES,
    RobotProfile,
    create_robot_kinematics,
    get_robot_profile,
    get_robot_urdf_path,
    list_robot_profiles,
)

__all__ = [
    "ARX_R5_LEFT_ARM_JOINT_NAMES",
    "ARX_R5_RIGHT_ARM_JOINT_NAMES",
    "DEFAULT_NERO_END_EFFECTOR_FRAME",
    "DEFAULT_NERO_JOINT_LIMITS",
    "DEFAULT_NERO_JOINT_NAMES",
    "FRANKA_PANDA_JOINT_LIMITS",
    "FRANKA_PANDA_JOINT_NAMES",
    "NeroKinematics",
    "RobotProfile",
    "create_robot_kinematics",
    "get_robot_profile",
    "get_robot_urdf_path",
    "list_robot_profiles",
]
