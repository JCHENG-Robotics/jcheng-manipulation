# Session 4 — Error Budget, Paired Trials and Host/DDS Benchmark

Date: 2026-11-17
Repo: `jcheng-manipulation` (Session 4 changes)
Scope: R11 + R12 + R13 + R15 from the remote work plan.

This bundle contains **offline/synthetic analysis** and **read-only hardware
measurements**. No robot motion, grasp, calibration capture or vendor demo was
executed.

## Contents

| Path | Content | Scope |
|---|---|---|
| `error_budget_demo.json/.md` | Synthetic complete budget with known values and distributions | Offline validation |
| `error_budget_baseline_report.json/.md` | Project budget with measured hand-eye/fiducial terms and unmeasured required terms | Offline analysis of hardware calibration/perception evidence |
| `paired_trials_plan.json` | 24-pair randomized schedule, seed 7 | Offline plan; not executed |
| `paired_trials_simulation.json` | Simulation input/output bundle | Synthetic |
| `paired_trials_simulated.jsonl` | 48 simulated records | Synthetic |
| `paired_trials_simulation_report.md` | Simulation analysis report | Synthetic |
| `paired_trials_analysis.json/.md` | Re-analysis of the simulated JSONL | Synthetic |
| `paired_trials_validation.json` | Schema/completeness validation | Synthetic |
| `no_camera/` | Host + ROS benchmark without the camera running | Real hardware, read-only |
| `with_camera/` | Host + ROS benchmark with the camera running | Real hardware, read-only |
| `commands.log` | Exact commands and test summaries | Record |
| `tests.txt` | Local colcon/pytest summary | Record |

## Key results

### Error budget (partial)

```text
translation  RSS 3.699 mm  worst case 4.980 mm  tolerance 5.000 mm
rotation     RSS 0.682 deg worst case 0.960 deg tolerance 2.000 deg
status: partial (7 required sources unmeasured)
```

Measured terms are the held-out hand-eye error (3.29 mm / 0.43°) and the
semi-synthetic fiducial error on real frames (1.69 mm / 0.53°). The 5 mm / 2°
tolerance is a provisional protocol gate. Do not read the RSS as a complete
task error.

### Paired trials (synthetic)

```text
24 pairs, true effect 1.0 mm
mean paired difference −1.025 mm, 95% CI [−1.095, −0.950] mm
sign test 24/0, p = 1.19e-07
success 75.0% -> 95.8%, McNemar p = 0.1250
```

The continuous effect is recovered; the 24-pair success comparison is
underpowered. No physical trial was run.

### Host/DDS benchmark

| Item | Result |
|---|---|
| Laptop host | Ryzen 7 5800H, 16 cores, 4.4–4.7% CPU, 27.9 GB, disk 14.3% |
| Pi host | Pi 5, 4 cores, 1.8% idle / 10.7% with camera, 8.05 GB, disk 84.7% (6.6 GB free), 46.9–48.0 °C |
| Ping | 0.239–0.254 ms avg, 0.281–0.298 ms max, 0% loss |
| `/joint_states` | 20.00 Hz, jitter 0.16 ms local / 0.30–0.60 ms cross-host |
| `/controller_manager/joint_states` | 48.8 Hz local / 48.5 Hz cross-host |
| Camera RGB | **~12 Hz cross-host**, 8.33 Hz in-container subscription, 7.0 Hz in-container `ros2 topic hz` |

The camera rate correction is important: the prior 8 Hz figure was an
in-container subscriber measurement and undercounts. In-container image
subscription also shows 104 ms jitter and a 313 ms p95, while the laptop sees
20.8 ms jitter and a 121 ms p95. See
[`../../../docs/phase3/HOST_BENCHMARK.md`](../../../docs/phase3/HOST_BENCHMARK.md).

## State changes and anomalies

- The camera launch was started for the `with_camera` run and then stopped.
  The camera publisher count returned to 0. The known reaping issue left a
  defunct `aurora930_node` process entry inside the container; it has no
  publisher. A container restart clears it.
- `get_node_names()` fails in the robot container with `empty node name
  returned by the RMW layer`. Topic discovery/subscription still works.
- Camera and static-TF logs repeat `serdata.cpp` errors (`string data is not
  null-terminated` / `invalid data size`). This is an open DDS/graph defect.
- The robot root filesystem is 84.7% used. No cleanup was performed (R14 is
  outside Session 4).
- `/joint_states` on the real robot remains a cached command stream, not
  independent encoder feedback.

## Commands

See [`commands.log`](commands.log). The benchmark collector is:

```bash
TOPICS=/joint_states,/depth_cam/rgb/image_raw,/ros_robot_controller/battery \
  tools/phase4/run_benchmark.sh evidence/phase3/session4_20261117/with_camera
```

## Rules

- Do not label the error budget or paired analysis as hardware validation.
- Do not quote the in-container camera FPS as the sensor rate; use the
  cross-host measurement and state the location.
- The simulated trial records are synthetic and must not be mixed with future
  hardware records.
