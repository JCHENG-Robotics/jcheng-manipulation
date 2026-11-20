# Fiducial → Grasp Bridge and Mock Visual-Grasp E2E

> Status: **Implemented and mock-validated (2026-11-17, Session 3).** The bridge
> consumes a live `target_pose`, transforms it to `base_link`, runs the 5-DoF
> grasp search and sends the selected plan to the safety veto. The marker,
> camera and robot are synthetic/mock; **nothing is executed through this
> bridge**. Real markerless grasps were later executed through the grasp
> planner on 2026-11-18/19 (see
> [`evidence/phase2/onsite_20261119/`](../../evidence/phase2/onsite_20261119/)),
> but the fiducial bridge itself remains mock-only.

## Why this exists

The grasp planner originally accepted a hand-typed `--target-point`. The
fiducial localizer already published `target_pose`, but nothing connected the
two. The bridge closes that gap and makes the frame contract explicit:

```text
synthetic/real image
→ AprilTag detection (armpi_fiducial_localizer)
→ target_pose in camera_optical_calibrated
→ TF to base_link
→ optional marker-frame grasp offset
→ joint-space grasp search (armpi_grasp_planner)
→ IK → FK → collision → plan-only Pilz PTP
→ safety veto (armpi_safety)
→ JSON report
```

No trajectory is sent to a controller. The safety service is a veto, not an
executor.

## Contract

| Item | Value | Notes |
|---|---|---|
| Input topic | `target_pose` (`geometry_msgs/PoseStamped`) | published by the localizer |
| Input frame | `camera_optical_calibrated` | measured optical frame; see [camera frames](../hardware/camera_frames.md) |
| Output frame | `base_link` | TF is looked up with a 0.2 s timeout |
| Pose age | ≤ 0.5 s | reject stale queued images |
| Marker orientation | +z within 45° of `base_link` +z | rejects wall-mounted or flipped markers |
| Grasp offset | zero by default | `--marker-offset X Y Z` in the **marker** frame; +z is the marker normal |
| Search | joint-space sampler | 80 candidates, 20 mm radius, pitch ≤ 75° |
| Safety | optional service | `/armpi_safety/validate_trajectory` |

The marker **origin** is used as the TCP point. The marker orientation is only
checked for tilt; it is not yet used to choose a grasp orientation or an object
centre. A marker placed on the side of an object therefore needs a deliberate
marker-frame offset.

## Running the mock E2E

The one-shot bridge node and the mock chain are separate so the bridge has an
explicit exit code and JSON report:

```bash
# Terminal 1 - MoveIt mock, synthetic marker, localizer, safety supervisor.
# The launch defaults to an isolated DDS domain (77, localhost only).
ros2 launch armpi_grasp_planner mock_visual_grasp.launch.py use_rviz:=false

# Terminal 2 - same isolated environment.
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1
ros2 run armpi_grasp_planner grasp_target_bridge \
  --safety-service /armpi_safety/validate_trajectory \
  --include-trajectory \
  --json-out /tmp/bridge.json
```

The one-command evidence runner starts and stops the chain itself:

```bash
tools/phase2/run_mock_visual_grasp_e2e.sh evidence/phase2/session3_20261117
```

It also runs `tools/phase2/safety_service_check.py` against the planned
trajectory: one valid case and three negative controls (joint limit, velocity,
timestamp).

> **Never run `mock_visual_grasp.launch.py` while the real ArmPi stack is up.**
> It starts mock controllers and a second `/joint_states`/`/tf` source. The
> isolated domain is a safety measure, not a licence to run both stacks in the
> same graph.

## Pose used in the mock run

The synthetic camera is at `(0.1382, 0.0225, 0.3496)` in `base_link`, looking
down (180° roll). The marker is 0.26 m in front of the camera, so the marker
centre lands at `(0.1382, 0.0225, 0.0896)`, the known reachable grasp point.
The marker is generated at `rvec=(π, 0, 0)` in the optical frame; after the
camera rotation the marker +z is `base_link` +z, as a flat, upward-facing
marker should be.

The localizer reported the marker at `t=(-0.0001, -0.0001, 0.2601)` with
0.00 px reprojection in the synthetic image. After TF the bridge measured:

```text
grasp_point = (0.138095, 0.022605, 0.089455) m
marker_tilt = 0.0035 deg
pose_age    = 0.061 s (gate 0.5 s; varies with the capture instant)
```

## Mock result (2026-11-17)

Evidence: [`evidence/phase2/session3_20261117/`](../../evidence/phase2/session3_20261117/).

| Item | Result |
|---|---|
| Target capture | PASS, `camera_optical_calibrated → base_link` |
| Candidates | 80 generated → 80 sanity → 40 IK → 40 FK → 26 collision-free |
| Plans | 3 plan-only Pilz `PTP`, all `PLAN_OK` |
| Safety | 3/3 plans approved, `code=OK` |
| Best candidate | `js_0249`, 87 points, 8.51 s trajectory duration |
| Max velocity / acceleration | 0.421 rad/s / 0.10 rad/s² |
| Workspace check | `end_effector_link`, `workspace_checked=true` |
| Bridge exit | 0 (`BRIDGE_RESULT PASS`) |
| E2E exit | 0 (`E2E PASS`) |

The safety service negative controls in the same run:

```text
SAFETY_CASE valid      approved=True  code=OK
SAFETY_CASE joint_limit approved=False code=JOINT_LIMIT
SAFETY_CASE velocity   approved=False code=VELOCITY_LIMIT
SAFETY_CASE timestamp  approved=False code=TIMESTAMP
RESULT PASS
```

Because the plan is plan-only, the "approved" trajectory was never executed.
The mock joint state is also not a hardware measurement.

## Limitations

- **No real camera, marker or object.** The image is rendered; the marker is
  not printed; there is no lighting, motion blur or lens model.
- **No real grasp through this bridge.** The selected trajectory is not
  executed; there is no gripper command, object contact or grasp-success
  measurement. The later real grasps used the grasp planner directly.
- **Marker centre = TCP.** The bridge does not segment the object or estimate
  its centre. `--marker-offset` is a manual correction in the marker frame.
- **Flat-marker assumption.** Markers tilted more than 45° are rejected; a
  marker on a vertical face is not supported yet.
- **No temporal filtering.** Each run uses one accepted pose; there is no
  multi-frame average or outlier rejection.
- **Safety is a veto layer.** It does not own the controller, the e-stop or a
  real-time deadline. The limits are adopted EEPROM-bounded values; since the
  2026-11-19 survey the mapping is `mapping_verified=true` for the operating
  envelope.
- **DDS isolation is mandatory.** Use the launch's default domain 77 or set
  `ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY` consistently for the launch, bridge and
  any `ros2` CLI inspection.

## Next hardware step

On the real robot, with the camera mounted and the hand-eye frame published:

1. Print a 36h11 marker with a measured 0.06 m edge and place it flat on the
   work surface.
2. Start the vendor camera/localizer on the robot side and confirm
   `target_pose` in `camera_optical_calibrated`.
3. Run the bridge with `--max-plan 1` and **no** execution path; verify the
   transformed point against a ruler/survey.
4. Only after the joint-limit survey and safety review, consider a supervised
   approach motion.
