from __future__ import annotations

import numpy as np

from pinocchio_kinematics_lite import pose_errors


def test_joint_limit_helpers_detect_violations(nero_kin):
    limits = nero_kin.get_joint_limits()
    q_mid = np.mean(limits, axis=1)

    assert nero_kin.check_joint_limits(q_mid)

    too_low = q_mid.copy()
    too_low[0] = limits[0, 0] - 0.01
    assert not nero_kin.check_joint_limits(too_low)

    too_high = q_mid.copy()
    too_high[-1] = limits[-1, 1] + 0.01
    assert not nero_kin.check_joint_limits(too_high)


def test_solver_clips_and_returns_solutions_inside_joint_limits(nero_kin):
    limits = nero_kin.get_joint_limits()
    mixed = np.where(np.arange(7) % 2 == 0, limits[:, 0] - 1.0, limits[:, 1] + 1.0)
    clamped = nero_kin.clip_to_joint_limits(mixed)

    assert nero_kin.check_joint_limits(clamped)

    q_target = nero_kin.sample_random_q(seed=13)
    target_T = nero_kin.forward_kinematics(q_target)
    result = nero_kin.inverse_kinematics(
        target_T,
        q_init=q_target,
        max_iters=40,
        pos_tol=1e-6,
        ori_tol=1e-6,
    )

    assert result.success, result.as_dict()
    assert result.q is not None
    assert nero_kin.check_joint_limits(result.q)
    pos_err, ori_err = pose_errors(nero_kin.forward_kinematics(result.q), target_T)
    assert pos_err < 1e-6
    assert ori_err < 1e-6
