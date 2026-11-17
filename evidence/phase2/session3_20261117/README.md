# Session 3 — Bridge, Mock Visual-Grasp E2E, Safety Veto, Measurement Protocols

> Scope: **mock/synthetic only.** No real camera, marker, robot motion or grasp
> was used. `mock_components` controllers and a rendered AprilTag image were
> used. The plan is plan-only; the safety service is a veto, not an executor.

Date: 2026-11-17 (evening)
Host: laptop, ROS 2 Jazzy, `mock_components`, DDS domain 77 + localhost only.
Repo: `jcheng-manipulation` (Session 3 changes).

## What ran

| Step | Command | Result |
|---|---|---|
| Package build | `colcon build --symlink-install --packages-select arm_interfaces armpi_grasp_planner armpi_safety armpi_fiducial_localizer` | 4 packages, no errors |
| Package tests | `colcon test --packages-select ...` | **58 tests, 0 failures** |
| Tool tests | `python3 -m pytest tools/phase2/tests -q` | **25 passed** |
| Mock visual-grasp E2E | `tools/phase2/run_mock_visual_grasp_e2e.sh evidence/phase2/session3_20261117` | **E2E PASS**, exit 0 |
| Safety negative controls | `tools/phase2/safety_service_check.py --input mock_visual_grasp_bridge.json` | **4/4 cases PASS** |
| Measurement dry-run/simulate | `tools/phase2/measurement_protocol.py <protocol> --mode ...` | 4 protocols, all PASS |

The mock chain was launched with
`ros2 launch armpi_grasp_planner mock_visual_grasp.launch.py use_rviz:=false`.
The launch defaults to `ROS_DOMAIN_ID=77` and `ROS_LOCALHOST_ONLY=1`; the E2E
runner also exports both so its `ros2` CLI checks use the same graph. An idle
Humble MoveIt container (`armpi-humble`) was stopped before the run to avoid a
second `/compute_fk`/`/move_action` server. The real robot stack was not used.

## E2E result

The synthetic marker is rendered in `camera_optical_calibrated`; the camera is
at `(0.1382, 0.0225, 0.3496)` in `base_link`, looking down. The marker centre is
0.26 m in front of the camera, so the transformed grasp point is the known
reachable point near `(0.1382, 0.0225, 0.0896)`.

| Item | Result |
|---|---|
| Target captured | `camera_optical_calibrated → base_link` |
| Captured point | `(0.138095, 0.022605, 0.089455)` m |
| Marker tilt / pose age | 0.0035° / 0.061 s (gate 0.5 s) |
| Candidates | 80 generated, 80 sanity, 40 IK, 40 FK, 26 collision-free |
| Plans | 3/3 `PLAN_OK` (plan-only Pilz PTP) |
| Safety | 3/3 approved, `code=OK` |
| Best | `js_0249`, 87 points, 8.51 s trajectory duration |
| Max velocity / acceleration | 0.421 rad/s / 0.10 rad/s² |
| Workspace | `end_effector_link` checked, inside the box |

Safety service cases:

```text
valid       approved=True  code=OK
joint_limit approved=False code=JOINT_LIMIT
velocity    approved=False code=VELOCITY_LIMIT
timestamp   approved=False code=TIMESTAMP
```

## Measurement protocol result

No hardware sample exists. The `simulate` reports use injected
`noise=0.3 mm`, `bias=1.0 mm`, `seed=7`.

| Protocol | Schedule | Simulated analysis |
|---|---|---|
| repeatability | 30 trials | radial RMS 0.483 mm, max 0.919 mm, nominal-error mean 1.130 mm |
| backlash | 30 trials | hysteresis 0.939 mm |
| visual displacement | 21 placements | RMSE 1.139 mm, max 1.876 mm |
| gripper envelope | 33 offsets | 6.0 mm on x/y/z (injected boundary) |

These validate the analysis path, not the robot.

## Files

| File | Content |
|---|---|
| `mock_visual_grasp_launch.txt` | full launch log (mock MoveIt + localizer + safety) |
| `mock_visual_grasp_nodes.txt`, `mock_visual_grasp_services.txt` | graph snapshot |
| `mock_visual_grasp_bridge.txt` | bridge/search console log |
| `mock_visual_grasp_bridge.json` | combined bridge report (capture, summary, best) |
| `mock_visual_grasp_search.json` | full `grasp_candidate_check` report, including the 87-point trajectory |
| `safety_service_check.txt` | valid + 3 negative control service cases |
| `mock_visual_grasp_result.txt` | `E2E PASS` |
| `measurement_protocol.txt` | dry-run/simulate console log |
| `measurement_*_dry_run.json` | schedules only, no ROS, no motion |
| `measurement_*_simulate.json` | synthetic samples and analysis |
| `test_results.txt` | colcon and pytest summaries |
| `SHA256SUMS` | hashes of this bundle |

## Limitations

- The image is rendered; the marker is not printed and no real camera/lighting
  is involved.
- The plan is not executed. There is no gripper command, object, contact or
  grasp-success measurement.
- The mock joint state is not a hardware measurement; `/joint_states` on the
  real robot is a cached command.
- Joint limits, velocity/acceleration limits and the workspace box are
  provisional.
- The safety layer does not own the controller or the e-stop and cannot cut
  power.
- Measurement numbers are synthetic; no repeatability/backlash/visual/gripper
  data exists yet.
