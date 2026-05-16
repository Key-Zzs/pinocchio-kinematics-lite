from __future__ import annotations

import numpy as np

from pinocchio_kinematics_lite import IKResult, pose_errors


def test_fk_output_contract(nero_kin):
    q = np.zeros(7)

    T = nero_kin.forward_kinematics(q)
    pose = nero_kin.fk_pose(q)

    assert T.shape == (4, 4)
    assert pose.shape == (6,)
    assert np.all(np.isfinite(T))
    assert np.all(np.isfinite(pose))
    assert np.allclose(T[3], np.array([0.0, 0.0, 0.0, 1.0]))
    assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-8)
    assert abs(np.linalg.det(T[:3, :3]) - 1.0) < 1e-8
    assert nero_kin.end_effector_frame == "link7"
    assert nero_kin.list_joints() == [f"joint{i}" for i in range(1, 8)]


def test_fk_ik_consistency_reachable_targets(nero_kin, rng):
    for seed in range(4):
        q_target = nero_kin.sample_random_q(seed=seed)
        q_init = nero_kin.clip_to_joint_limits(q_target + rng.normal(0.0, 0.02, size=7))
        target_T = nero_kin.forward_kinematics(q_target)

        result = nero_kin.inverse_kinematics(
            target_T,
            q_init=q_init,
            max_iters=160,
            pos_tol=1e-5,
            ori_tol=1e-4,
        )

        assert isinstance(result, IKResult)
        assert result.success, result.as_dict()
        assert result.q is not None
        actual_T = nero_kin.forward_kinematics(result.q)
        pos_err, ori_err = pose_errors(actual_T, target_T)
        assert pos_err < 1e-3, result.as_dict()
        assert ori_err < 1e-2, result.as_dict()
