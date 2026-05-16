from __future__ import annotations

import sys

import numpy as np

from pinocchio_kinematics_lite import NeroKinematics, pose_errors


def test_nero_profile_initializes_without_vendor_sdk(pinocchio_available):
    kin = NeroKinematics()

    assert "pyAgxArm" not in sys.modules
    assert "zerorpc" not in sys.modules
    assert kin.list_joints() == [f"joint{i}" for i in range(1, 8)]
    assert kin.end_effector_frame == "link7"


def test_nero_fk_ik_jacobian_basic(nero_kin):
    q = np.zeros(7)
    T = nero_kin.forward_kinematics(q)
    J = nero_kin.jacobian(q)
    result = nero_kin.inverse_kinematics(T, q_init=q)

    assert J.shape == (6, 7)
    assert result.success, result.as_dict()
    assert result.q is not None
    pos_err, ori_err = pose_errors(nero_kin.forward_kinematics(result.q), T)
    assert pos_err < 1e-6
    assert ori_err < 1e-6


def test_nero_default_urdf_is_not_vendor_fallback(pinocchio_available, monkeypatch):
    monkeypatch.delenv("NERO_URDF_PATH", raising=False)
    nero_kin = NeroKinematics()
    normalized = nero_kin.urdf_path.replace("\\", "/")
    resolved = nero_kin.resolved_urdf_path.replace("\\", "/")
    assert "pyAgxArm" not in normalized
    assert "asserts/agx_arm_urdf" not in normalized
    assert f"/{'home'}/" not in normalized
    assert normalized.endswith("assets/nero/nero_description.urdf")
    assert "pyAgxArm" not in resolved
    assert "asserts/agx_arm_urdf" not in resolved
    assert f"/{'home'}/" not in resolved
    assert resolved.endswith("assets/nero/nero_description.urdf")
