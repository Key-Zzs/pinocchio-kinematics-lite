#!/usr/bin/env python3
"""Debug FK/IK for one reachable target."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from pinocchio_kinematics_lite import NeroKinematics, PinocchioKinematics, pose_errors


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-profile", choices=("nero",), default="nero")
    parser.add_argument("--urdf-path", type=Path, default=None)
    parser.add_argument("--end-effector-frame", default=None)
    parser.add_argument("--active-joint-names", nargs="*", default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--q", type=float, nargs="*", default=None, help="Target active-joint vector")
    parser.add_argument("--q-init", type=float, nargs="*", default=None, help="IK initial active-joint vector")
    parser.add_argument("--max-iters", type=int, default=100)
    parser.add_argument("--pos-tol", type=float, default=1e-4)
    parser.add_argument("--ori-tol", type=float, default=1e-3)
    return parser.parse_args()


def make_kinematics(args):
    if args.urdf_path is not None:
        if args.end_effector_frame is None:
            raise SystemExit("--end-effector-frame is required when --urdf-path is used")
        return PinocchioKinematics(
            urdf_path=args.urdf_path,
            end_effector_frame=args.end_effector_frame,
            active_joint_names=args.active_joint_names,
        )
    return NeroKinematics(end_effector_frame=args.end_effector_frame or "link7")


def main():
    args = parse_args()
    kin = make_kinematics(args)
    rng = np.random.default_rng(args.seed)

    q_target = np.asarray(args.q, dtype=float) if args.q is not None else kin.sample_random_q(seed=args.seed)
    q_target = kin.clip_to_joint_limits(q_target)
    q_init = (
        np.asarray(args.q_init, dtype=float)
        if args.q_init is not None
        else kin.clip_to_joint_limits(q_target + rng.normal(0.0, 0.03, size=q_target.shape))
    )

    target_T = kin.forward_kinematics(q_target)
    result = kin.inverse_kinematics(
        target_T,
        q_init=q_init,
        max_iters=args.max_iters,
        pos_tol=args.pos_tol,
        ori_tol=args.ori_tol,
    )

    pos_err = None
    ori_err = None
    if result.q is not None:
        pos_err, ori_err = pose_errors(kin.forward_kinematics(result.q), target_T)

    payload = {
        "urdf_path": kin.urdf_path,
        "resolved_urdf_path": getattr(kin, "resolved_urdf_path", kin.urdf_path),
        "end_effector_frame": kin.end_effector_frame,
        "joint_names": kin.list_joints(),
        "q_target": q_target.tolist(),
        "q_init": q_init.tolist(),
        "ik_result": result.as_dict(),
        "position_error": pos_err,
        "orientation_error": ori_err,
        "joint_limit_violation": None if result.q is None else not kin.check_joint_limits(result.q),
    }
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
