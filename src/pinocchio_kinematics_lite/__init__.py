"""Pinocchio Kinematics Lite."""

from .ik import IKResult
from .kinematics import PinocchioKinematics
from .profiles.nero import (
    DEFAULT_NERO_END_EFFECTOR_FRAME,
    DEFAULT_NERO_JOINT_LIMITS,
    DEFAULT_NERO_JOINT_NAMES,
    DEFAULT_NERO_URDF_RESOURCE,
    NeroKinematics,
)
from .profiles.registry import (
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
from .resources import get_nero_urdf_path
from .transforms import (
    as_pose_matrix,
    pose_errors,
    pose_matrix_from_xyz_quat,
    pose_matrix_from_xyz_rpy,
    xyz_quat_from_pose_matrix,
    xyz_rpy_from_pose_matrix,
)

__all__ = [
    "ARX_R5_LEFT_ARM_JOINT_NAMES",
    "ARX_R5_RIGHT_ARM_JOINT_NAMES",
    "DEFAULT_NERO_END_EFFECTOR_FRAME",
    "DEFAULT_NERO_JOINT_LIMITS",
    "DEFAULT_NERO_JOINT_NAMES",
    "DEFAULT_NERO_URDF_RESOURCE",
    "FRANKA_PANDA_JOINT_LIMITS",
    "FRANKA_PANDA_JOINT_NAMES",
    "IKResult",
    "NeroKinematics",
    "PinocchioKinematics",
    "RobotProfile",
    "as_pose_matrix",
    "create_robot_kinematics",
    "get_nero_urdf_path",
    "get_robot_profile",
    "get_robot_urdf_path",
    "list_robot_profiles",
    "pose_errors",
    "pose_matrix_from_xyz_quat",
    "pose_matrix_from_xyz_rpy",
    "xyz_quat_from_pose_matrix",
    "xyz_rpy_from_pose_matrix",
]

__version__ = "0.1.0"
