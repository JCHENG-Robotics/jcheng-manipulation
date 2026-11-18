# First real markerless RGB-D grasp — 2026-11-18

> Status: **PASS with caveats.** A 39.4 mm wooden cube on the floor was grasped
> and lifted off the ground with the two-finger gripper, using an
> **AprilTag-free** point-cloud segmentation. The successful grasp was the
> **second attempt**: the first, fully autonomous placement missed by a few
> millimetres; the second used the operator to slide the cube between the
> already-positioned open fingers. The camera was down for the success record
> (three failed restarts), so the visual evidence is the operator photo
> [`operator_grasp_20261118.jpg`](operator_grasp_20261118.jpg), not a recorder
> frame.

## Goal

Grasp the 39.4 mm cube lying on the floor in front of the robot with **no
fiducial marker**: segment it from the RGB-D cloud, compute its 3-D centre,
plan a grasp with `armpi_grasp_planner`, execute the inspected trajectory, close
the gripper and lift.

## Setup

| Item | Value |
|---|---|
| Robot | ArmPi Ultra / Pi 5, vendor stack in container `ArmPiUltra` |
| Laptop stack | Humble container `armpi-humble`: `move_group`, `armpi_safety`, RViz |
| Object | wooden cube, **39.4 mm** edge, ~40 g, no marker |
| Support surface | marble floor, measured plane **z ≈ −0.079 m** in `base_link` |
| Camera | Aurora 930 RGB 640×400, cloud `/depth_cam/depth_registered/points` |
| Hand-eye | table-view TSAI, `evidence/calibration/20261118/table_down/` |
| Safety | `/armpi_safety/validate_trajectory` (table plane clearance 10 mm) |
| Operator | at the robot, power switch within reach |

## Geometry — the decisive measurements

| Quantity | Value | Consequence |
|---|---|---|
| Floor plane (measured) | **z = −0.0787 m** (ruler) / −0.079…−0.082 (cloud, pose-dependent) | — |
| Cube top | **z = −0.039…−0.040 m** | — |
| Lowest reachable **TCP** at the cube-top height | **x = 0.233 m**, with `joint2 = −0.6618`, `joint3 = −1.8012`, `joint4 = −0.3917` (all at the adopted EEPROM bounds) and `--joint-limit-margin 0.0` | the default 0.03 rad margin produces **no plan** |
| Camera near-edge at `table_down` | floor visible only from **x ≈ 0.24–0.26** | at the reach limit the cube is **outside** the camera view |
| Solution | a separate **view pose** `[0.504, 0.685, −1.652, −1.559, 1.383]` (range 0.225 m, gripper 0.143 m clear of the cube) then the grasp pose | "look, then grasp" |
| Reachable grid TCP (the only low pose) | **(0.2323, 0.0008, −0.0396)** | the placement tolerance is ±4.1 mm per side (47.6 mm open gap vs 39.4 mm cube) |

The reach limit was found by random joint-space FK over the adopted limits with
the 0.03 rad margin: the global minimum TCP z is ≈ −0.038 m, while the cube top
is at −0.040 m. With the margin removed the planner finds 3 plans at the bound;
the adopted limits themselves already carry the 20-pulse EEPROM margin.

## Markerless pipeline

New tool [`tools/phase2/object_segment_rgbd.py`](../../../../tools/phase2/object_segment_rgbd.py)
(container-side, numpy only; no fiducials):

1. capture one registered cloud and the **measured bus-servo pulses**;
2. transform with the measured hand-eye (optical convention);
3. RANSAC the support plane and remove it;
4. voxel-grid Euclidean connected components (5 mm pitch, 26-neighbour);
5. select the cluster by size/height/region gates;
6. fit the highest 5 mm slab: centroid, SVD tilt, min-area rectangle.

Validation on a **synchronised** RGB-D pair at the same instant:

| Quantity | Cloud | RGB (independent) | Difference |
|---|---|---|---|
| object top centre | (0.3081, 0.0004, −0.0428) m | (0.3062, 0.0045, −0.0393) m | 1.9 / −4.1 / −3.5 mm |
| top-face extent | 39.0 × 41.0 mm | — | cube is 39.4 mm |
| floor plane | 178,129 inliers, **0.75 mm RMS**, 0.12° to +z | — | — |

An earlier 65 mm cloud-vs-RGB disagreement was traced to the cube being **moved
between the two captures**, not to a calibration error.

## Attempt 1 — autonomous placement (miss)

Chassis approach (no odometry; measured 39 mm/s at 10 % duty, ~50 % on the
marginal strafe/backward pulses), then re-segmentation after each pulse:

| Step | Result |
|---|---|
| forward 0.8 s | cube x 0.308 → 0.277 |
| forward 0.9 s | cube left the camera's near edge (0 points above floor) |
| strafe 0.28 s + backward 0.16 s + backward 0.12 s | cube (0.2353, 0.0061, −0.0403), extent 37.8 × 31.9 mm (**partial view**) |
| view pose → grasp plan | 3 plans, TCP (0.2325, 0.0007, −0.0395), opening-axis offset **0.6 mm**, pad-length 4.9 mm |
| execution | grasp pose reached: pulses **501/342/929/407/766**; actual TCP (0.2334, 0.0017, −0.0391) |
| finger clearance | open finger min z = **−0.0717** (7.8 mm above the floor); pads straddle ~31 mm of the cube |
| close to −0.68 → lift joint2 +0.15 | actions `status=4`, but **the cube was not held** (operator: offset, no grasp) |

Cause: the last segmentation was a **partial top view** (37.8 × 31.9 mm vs 39.4),
so its centroid was biased by a few millimetres; combined with the 4.9 mm
pad-length offset and the planner's 0.01 rad goal tolerance the effective offset
exceeded the ±4.1 mm straddle allowance. The plan itself was valid — the
*material* was mislocated.

## Incident — DDS data plane loss and vendor restart

After the closed-loop work the laptop stopped receiving **all** robot data
(`/joint_states`, battery, camera) while DDS discovery still showed the
publishers, and `move_group` reported `Failed to receive current joint state`.
The cause was the **office WiFi path** used by both DDS configs
(`wlp4s0 10.51.128.246 ↔ wlan0 10.51.128.226`): 100 % packet loss, while the
wired link `10.42.0.218` was healthy.

Fix applied (both ends backed up as `*.wifi.bak`):

- laptop `~/jcheng/humble_ws/cyclonedds-wired.xml` → `enx00e04ca373c7` (10.42.0.1);
- robot `/home/ubuntu/cyclonedds.xml` → `eth0` (10.42.0.218);
- restart the vendor stack and the laptop `move_group` + `armpi_safety`.

**Operational lesson: restarting `start_node.service` re-initialises the
servos.** The arm went from the folded grasp pose to the vendor init pose
**700/500/500/500/500** (`joint1 = 0.8378 rad`, others 0) on its own; it does
**not** hold the previous posture. Plan the restart only at a safe pose.

The camera did not recover: three restarts of `armpi-vision.service` ended in
`aurora930_node` `open device failed 0x21003` (a stale handle), and the last
attempt left the node advertising but publishing nothing. Recovery needs a
vendor-container restart; retrying was stopped after three failures.

## Attempt 2 — operator placement (PASS)

After the DDS fix the arm was re-sequenced from the reset pose:

1. `table_down`-style view pose → plan/execute `status=4`;
2. grasp pose (open gripper) → plan/execute `status=4`; pulses
   **622/662/892/128/829** at the view pose, then the grasp pose;
3. **operator slides the cube between the open fingers** (47.6 mm gap,
   pads 8 mm above the floor) and steps back;
4. close to **−0.63** over 2.5 s → `status=4`;
5. lift `joint2 +0.15` (TCP +2.5 cm) → `status=4`, pulses
   **501/379/927/407/763**.

Result: **the cube was held and lifted off the floor** (operator confirmation
and [`operator_grasp_20261118.jpg`](operator_grasp_20261118.jpg)). The cube was
then lowered back to the grasp height and the gripper commanded open
(`r_joint −0.83`) — **but the cube did not release**. The arm was raised 3.4 cm
and returned to `table_down` (measured **499/568/775/130/496**) with the cube
still in the fingers.

## Release jam and torque release

`/joint_states` echoed `r_joint = −0.8336` (open) and every open goal returned
`status=4`, but the pads stayed clamped on the cube: **the commanded value is
not a physical confirmation**. A three-cycle −0.78 ↔ −0.86 wiggle with the
operator pulling did not free it either. The gripper servo (ID 1) torque was
then **released** with the vendor's inverted boolean
(`enable_torque=[1, 1]` / 0x0B), the operator opened the fingers by hand and
removed the cube, torque was re-engaged (`[1, 0]` / 0x0C), and the gripper was
commanded back to −0.80.

Lessons:

- a release must be **visually confirmed** (or read from the servo register)
  before any arm motion; `/joint_states` is the command, not the fingers;
- a 39.4 mm cube is close to this gripper's maximum opening (47.6 mm), and the
  pads/teeth can jam on it. Closing to −0.63 (about 0.6 mm under the object
  width) is enough to grip but makes release unreliable;
- the `GetBusServoState` service did not answer this session (it has no internal
  timeout), so the servo-register readback was unavailable; the torque *topic*
  path did work.

## What this proves and what it does not

Proves:

- a markerless RGB-D cloud segmentation can locate a 39 mm object on the floor
  to a few millimetres and feed the grasp planner;
- the whole chain — segmentation → plan → safety veto → exact trajectory →
  measured completion → gripper close → lift — works on the real robot;
- the reach boundary numbers are real: the grasp is only possible at the
  adopted joint bounds and with `--joint-limit-margin 0.0`.

Does **not** prove:

- a fully autonomous perception-to-grasp: the successful attempt was placed by
  the operator; the autonomous attempt missed;
- camera-recorded success: the camera was down;
- a repeatable grasp: one object, one pose, no force/contact feedback, no
  measured slip margin.

## Follow-ups (in priority order)

1. **Reject partial top views** in `object_segment_rgbd.py`: require the
   min-area extent ≥ 35 mm on both axes before trusting the centroid, and
   re-view otherwise. This is the direct cause of the attempt-1 miss.
   **Implemented 2026-11-18**: `--min-top-extent` (default 35 mm) with
   `top_view_gate`; a rejected view writes `rejection.reason =
   partial_top_view` and no `grasp_point_base` (10 unit tests).
2. **Last-millimetre placement**: use a finer `joint1` grid or an iterative
   "segment → nudge → segment" loop; the current 0.2 rad grid moves the TCP by
   ~46 mm per yaw step.
3. **Camera recovery**: vendor-container restart (device `0x21003`).
4. **DDS runbook**: record the wired-fallback procedure and that the WiFi path
   can fail while both nodes are up.
5. **Vendor restart behaviour**: add the servo re-init to
   `docs/hardware/ros_environment.md`; never restart the stack at a pose where
   the reset move is unsafe.
6. If the floor grasp is to be repeated, consider a raised object: the reach
   margin at the floor is zero and the grip depth is only ~31 mm.
7. **Release verification**: require a servo-register change or an operator
   confirmation before any motion after a gripper open; add it to the gripper
   helper rather than trusting the action status.
8. **Wider-opening or compliant pads** for objects near 40 mm, or grip closer to
   the object width so the pads do not wedge.

## Files

| File | Content |
|---|---|
| `seg4.json` | full-view segmentation at the view pose (39.0 × 41.0 mm) |
| `seg2.json`, `seg6.json`, `seg7.json` | successive post-chassis-pulse segmentations (the last is the partial-view 37.8 × 31.9 mm) |
| `floor_grasp2.json`, `floor_grasp3.json` | plan-only grasp searches at the measured cube positions |
| `operator_grasp_20261118.jpg` | operator photo of the held cube (off the floor) |
| `../../../../tools/phase2/object_segment_rgbd.py` | markerless segmentation tool |
| `../../../../tools/phase2/tests/test_object_segment.py` | 8 unit tests (clustering, min-area rect, slab fit, gates) |
| `../../../../tools/phase2/real_moveit_execute.py` | added `--joints` joint-target planning/execution |

Raw camera frames are not committed (they can contain people).
