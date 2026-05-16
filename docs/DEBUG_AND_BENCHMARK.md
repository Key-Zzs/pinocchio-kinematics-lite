# Debug And Benchmark

## Tests

```bash
pytest tests -v
```

Focused checks:

```bash
pytest tests/test_custom_urdf_loading.py -v
pytest tests/test_fk_ik_consistency.py -v
pytest tests/test_jacobian_consistency.py -v
pytest tests/test_joint_limits.py -v
pytest tests/test_nero_profile.py -v
pytest tests/test_no_vendor_dependency.py -v
```

## IK Benchmark

Nero profile:

```bash
python benchmarks/benchmark_ik.py \
  --robot-profile nero \
  --num-samples 100 \
  --output results/ik_benchmark_nero_100.json \
  --log-failures results/ik_failures_nero_100.jsonl
```

Custom URDF:

```bash
python benchmarks/benchmark_ik.py \
  --urdf-path path/to/robot.urdf \
  --end-effector-frame tool0 \
  --num-samples 100 \
  --output results/ik_benchmark_custom_100.json
```

## Trajectory Continuity Benchmark

Nero profile:

```bash
python benchmarks/benchmark_trajectory_continuity.py \
  --robot-profile nero \
  --num-samples 300 \
  --output results/trajectory_benchmark_nero_300.json
```

Custom URDF:

```bash
python benchmarks/benchmark_trajectory_continuity.py \
  --urdf-path path/to/robot.urdf \
  --end-effector-frame tool0 \
  --num-samples 300 \
  --output results/trajectory_benchmark_custom_300.json
```

## Debug One Target

```bash
python examples/debug_single_target.py --robot-profile nero --seed 7
```

For a custom URDF:

```bash
python examples/debug_single_target.py \
  --urdf-path path/to/robot.urdf \
  --end-effector-frame tool0 \
  --seed 7
```

## Metrics

- `success_rate`: fraction of samples whose IK result passed position, orientation, and joint-limit checks
- `timeout_rate`: fraction of samples that reached `max_iters`
- `position_error`: Euclidean translation error in meters
- `orientation_error`: geodesic SO(3) angle error in radians
- `p95_latency_ms` and `p99_latency_ms`: latency tail for solver calls
- `configuration_jump_count`: number of consecutive IK solutions whose joint-space step exceeded `--jump-threshold`

For redundant arms, do not expect `q_solution == q_target`. Evaluate IK by checking `FK(q_solution)` against the target pose.
