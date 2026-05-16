# Debug And Benchmark

## Tests

```bash
pytest tests -v
```

Useful focused checks:

```bash
pytest tests/test_no_vendor_dependency.py -v
pytest tests/test_custom_urdf_loading.py -v
pytest tests/test_nero_profile.py -v
```

## IK Benchmark

```bash
python benchmarks/benchmark_ik.py \
  --robot-profile nero \
  --num-samples 1000 \
  --seed 0 \
  --max-iters 100 \
  --pos-tol 1e-4 \
  --ori-tol 1e-3 \
  --output results/ik_benchmark_1000.json \
  --log-failures results/ik_failures_1000.jsonl
```

Custom URDF:

```bash
python benchmarks/benchmark_ik.py \
  --urdf-path path/to/robot.urdf \
  --end-effector-frame tool0 \
  --num-samples 200
```

## Trajectory Continuity Benchmark

```bash
python benchmarks/benchmark_trajectory_continuity.py \
  --robot-profile nero \
  --num-samples 300 \
  --output results/trajectory_benchmark_300.json \
  --log-failures results/trajectory_failures_300.jsonl
```

## Debug One Target

```bash
python examples/debug_single_target.py --robot-profile nero --seed 7
```

## Tolerances

Strict tolerances, such as `pos_tol=1e-5` and `ori_tol=1e-4`, are useful for regression tests on reachable targets generated from FK.

Deployment tolerances are often looser, such as `pos_tol=1e-4` to `1e-3` and `ori_tol=1e-3` to `1e-2`, depending on robot calibration, TCP definition, and downstream controller behavior.

## Metrics

- `success_rate`: fraction of samples whose IK result passed position, orientation, and joint-limit checks
- `timeout_rate`: fraction of samples that reached `max_iters`
- `position_error`: Euclidean TCP translation error in meters
- `orientation_error`: geodesic SO(3) angle error in radians
- `p95_latency_ms` / `p99_latency_ms`: latency tail for solver calls
- `configuration_jump_count`: number of consecutive IK solutions whose joint-space step exceeded `--jump-threshold`

For redundant 7-DoF arms, do not expect `q_solution == q_target`. Evaluate IK by checking `FK(q_solution)` against the target pose.
