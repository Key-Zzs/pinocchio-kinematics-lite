#!/usr/bin/env python3
"""Benchmark reachable-target IK quality for generic URDF kinematics."""

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

from pinocchio_kinematics_lite import (
    PinocchioKinematics,
    create_robot_kinematics,
    list_robot_profiles,
    pose_errors,
)


def scalar_stats(values) -> dict[str, float]:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return {
            "mean": float("nan"),
            "median": float("nan"),
            "p90": float("nan"),
            "p95": float("nan"),
            "p99": float("nan"),
            "max": float("nan"),
        }
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "p99": float(np.percentile(arr, 99)),
        "max": float(np.max(arr)),
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-profile", choices=list_robot_profiles(include_aliases=True), default="nero")
    parser.add_argument("--urdf-path", type=Path, default=None)
    parser.add_argument("--end-effector-frame", default=None)
    parser.add_argument("--root-frame", default=None)
    parser.add_argument("--active-joint-names", nargs="*", default=None)
    parser.add_argument("--locked-joint-names", nargs="*", default=None)
    parser.add_argument("--num-samples", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-iters", type=int, default=100)
    parser.add_argument("--pos-tol", type=float, default=1e-4)
    parser.add_argument("--ori-tol", type=float, default=1e-3)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument(
        "--log-failures",
        nargs="?",
        const=Path("ik_failures.jsonl"),
        default=None,
        type=Path,
        help="Optional JSONL path. Defaults to ik_failures.jsonl when passed without a value.",
    )
    return parser.parse_args()


def make_kinematics(args):
    if args.urdf_path is not None:
        if args.end_effector_frame is None:
            raise SystemExit("--end-effector-frame is required when --urdf-path is used")
        return PinocchioKinematics(
            urdf_path=args.urdf_path,
            end_effector_frame=args.end_effector_frame,
            root_frame=args.root_frame,
            active_joint_names=args.active_joint_names,
            locked_joint_names=args.locked_joint_names,
        )
    return create_robot_kinematics(
        args.robot_profile,
        end_effector_frame=args.end_effector_frame,
        root_frame=args.root_frame,
        active_joint_names=args.active_joint_names,
        locked_joint_names=args.locked_joint_names,
    )


def failure_record(index, q_target, q_init, result, pos_err, ori_err, reason):
    return {
        "index": int(index),
        "q_target": np.asarray(q_target, dtype=float).tolist(),
        "q_init": np.asarray(q_init, dtype=float).tolist(),
        "success": bool(result.success),
        "q_solution": None if result.q is None else np.asarray(result.q, dtype=float).tolist(),
        "best_q": None if result.best_q is None else np.asarray(result.best_q, dtype=float).tolist(),
        "last_q": None if result.last_q is None else np.asarray(result.last_q, dtype=float).tolist(),
        "position_error": None if pos_err is None else float(pos_err),
        "orientation_error": None if ori_err is None else float(ori_err),
        "iterations": int(result.iterations),
        "reason": reason,
        "ik_result": result.as_dict(),
        "solve_time_ms": float(result.solve_time_ms),
    }


def main():
    args = parse_args()
    kin = make_kinematics(args)
    rng = np.random.default_rng(args.seed)

    position_errors = []
    orientation_errors = []
    iterations = []
    latencies_ms = []
    failure_records = []
    success_count = 0
    timeout_count = 0
    joint_limit_violation_count = 0

    for idx in range(args.num_samples):
        q_target = kin.sample_random_q(seed=int(rng.integers(0, np.iinfo(np.int32).max)))
        target_T = kin.forward_kinematics(q_target)
        q_init = kin.clip_to_joint_limits(q_target + rng.normal(0.0, 0.03, size=q_target.shape))

        result = kin.inverse_kinematics(
            target_T,
            q_init=q_init,
            max_iters=args.max_iters,
            pos_tol=args.pos_tol,
            ori_tol=args.ori_tol,
        )
        latencies_ms.append(result.solve_time_ms)
        iterations.append(result.iterations)
        if result.reason == "max_iterations":
            timeout_count += 1

        pos_err = None
        ori_err = None
        if result.q is not None:
            pos_err, ori_err = pose_errors(kin.forward_kinematics(result.q), target_T)
            position_errors.append(pos_err)
            orientation_errors.append(ori_err)
            if not kin.check_joint_limits(result.q):
                joint_limit_violation_count += 1

        ok = (
            result.success
            and pos_err is not None
            and ori_err is not None
            and pos_err <= args.pos_tol
            and ori_err <= args.ori_tol
            and result.q is not None
            and kin.check_joint_limits(result.q)
        )
        if ok:
            success_count += 1
        else:
            reason = result.reason if not result.success else "tolerance_or_joint_limit"
            failure_records.append(failure_record(idx, q_target, q_init, result, pos_err, ori_err, reason))

    latency_stats = scalar_stats(latencies_ms)
    pos_stats = scalar_stats(position_errors)
    ori_stats = scalar_stats(orientation_errors)

    results = {
        "robot_profile": args.robot_profile if args.urdf_path is None else "custom_urdf",
        "urdf_path": kin.urdf_path,
        "resolved_urdf_path": getattr(kin, "resolved_urdf_path", kin.urdf_path),
        "end_effector_frame": kin.end_effector_frame,
        "root_frame": kin.root_frame,
        "joint_names": kin.list_joints(),
        "num_samples": int(args.num_samples),
        "success_count": int(success_count),
        "success_rate": float(success_count / max(1, args.num_samples)),
        "mean_position_error": pos_stats["mean"],
        "median_position_error": pos_stats["median"],
        "max_position_error": pos_stats["max"],
        "mean_orientation_error": ori_stats["mean"],
        "median_orientation_error": ori_stats["median"],
        "max_orientation_error": ori_stats["max"],
        "mean_iterations": float(np.mean(iterations)) if iterations else float("nan"),
        "max_iterations": int(np.max(iterations)) if iterations else 0,
        "mean_latency_ms": latency_stats["mean"],
        "median_latency_ms": latency_stats["median"],
        "p90_latency_ms": latency_stats["p90"],
        "p95_latency_ms": latency_stats["p95"],
        "p99_latency_ms": latency_stats["p99"],
        "max_latency_ms": latency_stats["max"],
        "timeout_rate": float(timeout_count / max(1, args.num_samples)),
        "joint_limit_violation_rate": float(joint_limit_violation_count / max(1, args.num_samples)),
        "failure_count": int(len(failure_records)),
    }

    print(json.dumps(results, indent=2, sort_keys=True))

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.log_failures:
        args.log_failures.parent.mkdir(parents=True, exist_ok=True)
        with args.log_failures.open("w", encoding="utf-8") as f:
            for record in failure_records:
                f.write(json.dumps(record, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
