from __future__ import annotations

import pytest

from pinocchio_kinematics_lite import (
    create_robot_kinematics,
    get_robot_profile,
    get_robot_urdf_path,
    list_robot_profiles,
)


def test_builtin_profile_registry_names():
    names = set(list_robot_profiles())

    assert {"nero", "franka_panda", "franka_panda_robotiq", "arx_r5", "arx_r5_left"} <= names

    names_and_aliases = set(list_robot_profiles(include_aliases=True))
    assert {"franka-panda", "franka-panda-robotiq", "arx_r5_right", "arx-r5-left"} <= names_and_aliases


def test_builtin_profile_defaults_are_chain_specific():
    panda = get_robot_profile("franka_panda")
    panda_robotiq = get_robot_profile("franka_panda_robotiq")
    arx_default = get_robot_profile("arx_r5")
    arx_left = get_robot_profile("arx_r5_left")

    assert panda.end_effector_frame == "link7"
    assert panda.active_joint_names == tuple(f"joint{i}" for i in range(1, 8))
    assert panda_robotiq.end_effector_frame == "robotiq_arg2f_base_link"
    assert panda_robotiq.active_joint_names == panda.active_joint_names
    assert panda_robotiq.joint_limits is not None
    assert panda_robotiq.joint_limits[-1] == pytest.approx((-2.8973, 2.8973))
    assert arx_default.active_joint_names == tuple(f"right_joint{i}" for i in range(1, 7))
    assert arx_default.root_frame == "right_base_link"
    assert arx_left.active_joint_names == tuple(f"left_joint{i}" for i in range(1, 7))
    assert arx_left.root_frame == "left_base_link"


def test_builtin_profile_urdf_paths_exist():
    for name in list_robot_profiles():
        assert get_robot_urdf_path(name).is_file()


def test_builtin_profile_factory_smoke(pinocchio_available):
    kin = create_robot_kinematics("franka_panda")

    assert kin.robot_profile == "franka_panda"
    assert kin.end_effector_frame == "link7"
    assert kin.list_joints() == [f"joint{i}" for i in range(1, 8)]
    assert kin.get_joint_limits()[-1].tolist() == pytest.approx([-2.8973, 2.8973])

