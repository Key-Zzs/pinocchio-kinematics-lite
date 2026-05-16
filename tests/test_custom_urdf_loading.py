from __future__ import annotations

import numpy as np

from pinocchio_kinematics_lite import PinocchioKinematics, get_nero_urdf_path, pose_errors


def test_explicit_urdf_path_loads_generic_kinematics(pinocchio_available):
    urdf_path = get_nero_urdf_path()
    kin = PinocchioKinematics(
        urdf_path=urdf_path,
        end_effector_frame="link7",
        active_joint_names=[f"joint{i}" for i in range(1, 8)],
    )

    q = np.zeros(7)
    T = kin.forward_kinematics(q)
    J = kin.jacobian(q)
    result = kin.inverse_kinematics(T, q_init=q, max_iters=20)

    assert T.shape == (4, 4)
    assert J.shape == (6, 7)
    assert result.success, result.as_dict()
    assert result.q is not None
    pos_err, ori_err = pose_errors(kin.forward_kinematics(result.q), T)
    assert pos_err < 1e-6
    assert ori_err < 1e-6
