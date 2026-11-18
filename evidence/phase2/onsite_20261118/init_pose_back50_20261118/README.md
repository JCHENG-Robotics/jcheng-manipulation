# init_pose + 50 cm backward base move — 2026-11-18 19:49

> Status: **arm at `init_pose` and base moved 50 cm backwards, both executed
> and read back.** No human displacement measurement yet: the base has no
> odometry, so the 50 cm is the calibrated duty-scale estimate, not a measured
> distance. The gripper was found **not fully open** and was re-referenced by
> hand; the documented open stop is wrong by ~200 register pulses — see
> [the gripper section](#gripper-must-be-fully-open--the-documented-open-stop-is-wrong).

## What was run

| Step | Command | Result |
|---|---|---|
| Arm to `init_pose` | `real_moveit_execute.py init_pose --execute --i-have-clear-workspace` (velocity scaling 0.05, joint4 clamped by limits) | `execute_status=4` (SUCCEEDED), `execute_error_code=1` |
| Base backwards | 3 × `chassis_jog.py --axis x --speed -0.167 --duration 1.0 --yes` | duty `[-30.394, +30.394, +30.394, -30.394]`, zero confirmed after each |
| Base hold | `chassis_jog.py --axis x --speed 0.0 --duration 1.0 --yes` | echoed command zero for IDs 1–4 |

## Arm read-back (`/joint_states`, `/controller_manager/servo_states` scale)

| Joint | Before | Commanded target | After |
|---|---|---|---|
| joint1 | −0.004189 | 0.0 | 0.004189 |
| joint2 | 0.284838 | 0.7309 | 0.737227 |
| joint3 | −1.151917 | −1.6127 | −1.608495 |
| joint4 | −1.549852 | −1.6591 | **−1.591740** |
| joint5 | −0.016755 | 0.0 | 0.0 |

- Plan: 62 waypoints, 6.046 s, `error_code=1`, safety `approved=True code=OK`
  (`/armpi_safety/validate_trajectory`).
- joint4 lands on the **adopted low bound −1.5917**, not the SRDF value
  −1.6591: `init_pose` in `armpi_ultra.srdf` is still outside
  `joint_limits.yaml` (known defect, see `joint_limit_margin_20261118.md`).
  The executed pose is therefore the limit-clamped init pose.
- joint2/3 residuals ≤ 0.006 rad (~1–2 servo pulses); joint5 exact.
- Arm pose unchanged across the base move (no sag), battery 6720 mV.

## Base distance

`duty = v · 182` and the 2026-11-18 measurement (duty 10 → 55 mm/s, duty 20 →
110 mm/s) give 0.167 m/s, so 3 × 1.0 s ≈ **0.501 m** commanded. Floor tile
slip, ramp-up and the 0.5 s watchdog tail are not modelled. **The physical
distance must be measured by hand** (record it here before using the table
calibration again).

## Gripper must be fully open — the documented open stop is wrong

The restore pose needs the gripper **fully open**. It was not: the operator
reported "a gap, but not maximum". Measured sequence (servo ID 1 is `r_joint`;
`servo_controller.yaml` maps joint1..joint5 to IDs 6..2):

| Step | Command | Register (linkage shaft) | Observation |
|---|---|---|---|
| start | — | 504 → 507 | gap, not maximum |
| "open" per SRDF (`r_joint −0.80`, raw 509) | `ServosPosition` | 504 | no change; already at the documented 502 stop |
| single open command | raw 500, 1.5 s | **stalls at 516** | cannot move toward 502 at all |
| close attempt | raw 632, 2.2 s | 521 | only 14 pulses of travel, then stalls |
| torque release (ID 1 only, `enable_torque=[1, 1]`) | — | 516, torque flag 0 | fingers free |
| **manual full open by the operator** | — | **299**, torque flag 0 | fingers reach the mechanical full-open stop |
| torque re-engage (`enable_torque=[1, 0]`) | — | **299**, torque flag 1, stable over 2.5 s | fingers hold fully open |

Findings:

- **The mechanical full-open position is register ≈ 299, not 502.**
  `docs/hardware/measured_joint_limits.md` records "open stop = command raw
  500, register 502". That value was the point where the servo stalled, not
  the full-open stop: the linkage was already binding ~14 pulses short, and
  hand force with torque released travels another ~217 register pulses.
- The command→register ratio measured on 2026-11-18 (raw 500→700 gives
  register 502→642) means register 299 corresponds to a command of roughly
  **raw 210**, i.e. `r_joint ≈ −2.05 rad`. The adopted `r_joint` range
  −0.8378…0.0 (raw 500…700) therefore covers only part of the finger travel.
- **Consequence:** any `open` goal (`r_joint −0.80` → raw 509) now *closes*
  the gripper from the true open position. Do not send `open` or the SRDF
  `open` state until the stop mapping is re-measured.
- The gripper was left fully open (`r_joint` readback still echoes −0.80, which
  is why `/joint_states` looked open while the fingers were not).

Related prior incident: [`first_grasp/README.md`](../first_grasp/README.md)
(release jam on the cube, torque released, fingers opened by hand). The same
release-then-hand-open procedure was used here on servo ID 1 only; the arm
joints kept torque throughout.

### Follow-up needed

1. ~~Re-measure both gripper stops in register terms~~ — **done** 2026-11-18
   19:59, see [`../gripper_stops_20261118/README.md`](../gripper_stops_20261118/README.md).
   Open stop is register 167 at raw ~150, not 502; the adopted open is raw 170
   (`r_joint = -2.22`).
2. ~~Recompute the `r_joint` ↔ raw mapping and update the limits/docs~~ —
   **done**: URDF, `joint_limits.yaml`, SRDF `open`, simulation initial
   position, planner `GRIPPER_OPEN_POSITION`, and the hardware docs. The
   laptop MoveIt workspace still needs a `colcon build` + relaunch for the
   running `move_group` to pick them up.
3. Check the rack/pinion for binding (the 14-pulse stall at 516 is a
   mechanical symptom) — **open**: the freed linkage now shows ~34 counts of
   backlash; inspect before grasping.

## Environment notes (not motion)

- The robot container `ArmPiUltra` did **not** have `moveit_msgs`, so
  `ros-humble-moveit-msgs` 2.2.3 (+ `object-recognition-msgs`,
  `octomap-msgs`) was installed into the running container over HTTP debs
  (the office WiFi is a captive portal, HTTPS 443 is unreachable from the
  robot). Ephemeral: lost when the container is recreated.
- `arm_interfaces` was built into a separate overlay
  `~/arm_interfaces_ws` on the robot (vendor workspace untouched) so the
  safety service type is available robot-side.
- `mecanum_chassis_node` was **not** running; it was started manually
  (`ros2 run controller mecanum`, duty scale 182, watchdog 0.5 s) and is left
  running.
- DDS: the robot container's nodes use `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  with `CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml` (eth0 wired). Any
  `docker exec` shell that omits either variable discovers the laptop's
  MoveIt nodes but cannot complete a service call — this cost a full
  debugging round trip here.
- The base move invalidates the table-plane calibration; the camera hand-eye
  link (`T_link4_camera`) is unaffected.
