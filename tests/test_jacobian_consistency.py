from __future__ import annotations

import numpy as np

from pinocchio_kinematics_lite.transforms import numerical_jacobian


def test_jacobian_matches_finite_difference(nero_kin):
    q = nero_kin.sample_random_q(seed=11)

    analytic_J = nero_kin.jacobian(q)
    numeric_J = numerical_jacobian(nero_kin.forward_kinematics, q, eps=1e-6)

    assert analytic_J.shape == (6, 7)
    assert numeric_J.shape == (6, 7)

    translational_inf_err = float(np.max(np.abs(analytic_J[:3] - numeric_J[:3])))
    rotational_inf_err = float(np.max(np.abs(analytic_J[3:] - numeric_J[3:])))

    assert translational_inf_err < 1e-4
    assert rotational_inf_err < 1e-3
