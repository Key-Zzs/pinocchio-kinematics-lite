#!/usr/bin/env python3
"""Minimal built-in Nero profile example."""

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pinocchio_kinematics_lite import NeroKinematics


kin = NeroKinematics()
q = np.zeros(7)

pose = kin.forward_kinematics(q)
J = kin.jacobian(q)
result = kin.inverse_kinematics(pose, q_init=q)

print("frames:", kin.list_frames()[-5:])
print("jacobian_shape:", J.shape)
print("ik_success:", result.success)
print("ik_q:", None if result.q is None else result.q.tolist())
