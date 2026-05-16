#!/usr/bin/env python3
"""Load an explicit URDF with the generic PinocchioKinematics API."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pinocchio_kinematics_lite import PinocchioKinematics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf_path", type=Path)
    parser.add_argument("--end-effector-frame", required=True)
    parser.add_argument("--active-joint-names", nargs="*", default=None)
    args = parser.parse_args()

    kin = PinocchioKinematics(
        urdf_path=args.urdf_path,
        end_effector_frame=args.end_effector_frame,
        active_joint_names=args.active_joint_names,
    )
    q = np.zeros(len(kin.list_joints()))
    pose = kin.forward_kinematics(q)
    result = kin.inverse_kinematics(pose, q_init=q)

    print("joints:", kin.list_joints())
    print("frames:", kin.list_frames())
    print("pose:\n", pose)
    print("ik:", result.as_dict())


if __name__ == "__main__":
    main()
