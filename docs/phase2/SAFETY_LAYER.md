# Conservative Safety Layer

> Status: **Implemented, mock-validated (2026-11-17) and wired into the real
> MoveIt execute path (2026-11-18).** The layer is a state watchdog and
> trajectory **veto**, not a controller or an e-stop. Joint limits now use the
> adopted EEPROM-bounded ranges (2026-11-18); mechanical limits are still only
> partially surveyed and no dedicated hardware safety test has been done.

## Why a veto layer first

The master plan requires conservative velocity/acceleration limits, validated
joint limits, execution timeout, a communication watchdog, workspace checks and
an accessible hardware stop before automated experiments. The hardware stop and
the mechanical limits need the robot and a person; the software veto can be
built and tested now.

The design separates:

- **pure policy** (`armpi_safety/policy.py`) — no ROS imports, unit-testable;
- **ROS supervisor** (`armpi_safety/node.py`) — subscribes `/joint_states`,
  publishes status/stop, exposes the validation service;
- **caller** — the grasp search (or a future executor) must refuse to execute
  when the service does not approve.

## ROS interface

| Interface | Type | Purpose |
|---|---|---|
| `armpi_safety/validate_trajectory` | `arm_interfaces/srv/ValidateTrajectory` | validate a planned trajectory; returns `approved`, `code`, `violations`, JSON report |
| `armpi_safety/reset_stop` | `std_srvs/srv/Trigger` | clear a latched state fault when the current state is valid |
| `armpi_safety/safety_status` | `std_msgs/String` (5 Hz) | JSON watchdog/validation status |
| `armpi_safety/safety_stop` | `std_msgs/Bool` (transient local) | fail-safe stop flag; true when the state is invalid or a fault is latched |

The service request carries `moveit_msgs/RobotTrajectory` and an optional
start-state threshold. Nothing in the node publishes a controller command.

## Checks

| Check | Rule | Failure code |
|---|---|---|
| State watchdog | latest `/joint_states` receipt ≤ 0.5 s old | `STATE_INVALID` |
| State completeness | `joint1…joint5` present and finite | `STATE_INVALID` |
| State limits | positions inside the adopted EEPROM-bounded joint ranges | `STATE_INVALID` / `JOINT_LIMIT` |
| Trajectory shape | arm joints present, positions length matches, ≤ 5000 points | `TRAJECTORY_INVALID` |
| Timestamps | strictly increasing | `TIMESTAMP` |
| Duration | ≤ 30 s | `DURATION` |
| Position limits | every waypoint in range (1e-3 rad tolerance) | `JOINT_LIMIT` |
| Velocities | explicit, else finite differences; ≤ 1.2 rad/s | `VELOCITY_LIMIT` |
| Accelerations | explicit, else finite differences; ≤ 0.5 rad/s² | `ACCELERATION_LIMIT` |
| Start state | first point within 0.08 rad of the latest state | `START_STATE` |
| Workspace | `end_effector_link` TCP inside the `base_link` box on every waypoint | `WORKSPACE` |
| Table clearance | TCP at least `clearance_m` above the measured table plane on every waypoint | `TABLE_CLEARANCE` |
| FK availability | fail closed when workspace or table FK is unavailable | `WORKSPACE` / `TABLE_CLEARANCE` |

A **real state fault latches** the stop: once stale/out-of-range/non-finite
state is seen, `safety_stop` stays true until `reset_stop` succeeds and the
current state is valid. A rejected **plan** is a veto, not a robot fault; it does
not latch the stop.

## Configuration

[`ros2_ws/src/armpi_safety/config/safety_limits.yaml`](../../ros2_ws/src/armpi_safety/config/safety_limits.yaml)
holds the adopted limits. The main values:

```yaml
joint_limits:      # EEPROM-bounded, 20-pulse margin (2026-11-18)
  joint1: {lower: -1.8012, upper: 1.8012}
  joint2: {lower: -0.6618, upper: 1.2985}
  joint3: {lower: -1.8012, upper: 0.0}
  joint4: {lower: -1.5917, upper: 1.5917}
  joint5: {lower: -1.5917, upper: 1.5499}
state: {timeout_s: 0.5}
start_state: {max_distance_rad: 0.08}
trajectory: {max_duration_s: 30.0, max_points: 5000}
workspace:
  frame: base_link
  enabled: true
  required: true
  lower: [-0.05, -0.30, -0.10]   # coarse hard floor, not the table guard
  upper: [0.45, 0.30, 0.55]
table_plane:                     # measured 2026-11-18
  enabled: true
  frame: base_link
  normal: [0.010305, -0.063014, 0.997959]
  offset: 0.0772828              # normal . p + offset = 0
  clearance_m: 0.01
```

The position bounds are the servo EEPROM limits minus a 20-pulse margin, not a
full mechanical survey. Velocity and acceleration (1.2 rad/s, 0.5 rad/s²) are
deliberately below the URDF's `10` values, and the planner additionally uses 0.1
scaling.

**Table envelope.** The table is at **z ≈ −0.078 m** and tilted **3.66°**, so
its height varies by about **33 mm** across `y = ±0.30 m`; a single flat z bound
cannot represent it. The workspace lower z is now a coarse **hard floor** at
`−0.10 m`, and the operative table guard is the `table_plane` check: the TCP is
rejected when its signed height above the measured plane is below
`clearance_m` (10 mm). The check is the calibrated lower bound the left-over
table issue asked for; it does **not** simply lower z without geometry.

Two limits of the plane check:

- It is **TCP-only**. Link3–5 and the gripper body need the table collision
  object; `tools/phase2/table_collision.py` publishes the measured plane as an
  oriented box on `/collision_object` (`table_surface`). Run it before the grasp
  search and keep it alive for the execute path.
- The plane was measured at one base pose. It is invalid after the mecanum
  chassis moves; re-measure the plane (or localize the table) before table work
  at a new base pose.

Reachability note: a random 120 000-sample FK search over the adopted limits
found a lowest TCP z of **−0.0445 m**, about **33 mm above the table**. The
table surface itself is not reachable by the TCP, so the 10 mm clearance is
satisfied by every reachable configuration; the check permits table grasps of
sufficiently tall objects while still rejecting an FK or command that would go
below the surface.

The real MoveIt path now calls the veto: `tools/phase2/real_moveit_execute.py`
validates the exact planned trajectory before `--execute`, and
`tools/phase2/execute_plan_json.py` does the same for a grasp-planner trajectory.
Both refuse execution when the service is unavailable or does not approve.

## Mock verification

Unit tests: `armpi_safety/test/test_policy.py` covers a valid trajectory,
stale state, joint/velocity/acceleration limits, start-state mismatch,
workspace excursion, table clearance (including the tilted plane and a missing
FK), missing joint, non-monotonic timestamps, duration, reset-stop semantics
and non-finite positions.

Service test on a real planned trajectory (mock MoveIt, plan-only):

```text
SAFETY_CASE valid      approved=True  code=OK
SAFETY_CASE joint_limit approved=False code=JOINT_LIMIT
SAFETY_CASE velocity   approved=False code=VELOCITY_LIMIT
SAFETY_CASE timestamp  approved=False code=TIMESTAMP
RESULT PASS
```

Table-envelope service test (isolated `ROS_DOMAIN_ID=42`, Jazzy mock, a
2-point trajectory whose TCP is at z = −0.0317 m, 47.9 mm above the measured
table plane):

```text
new config (table on, z floor -0.10)       approved=True  code=OK
old config (z floor 0.00, table off)       approved=False code=WORKSPACE
strict test config (clearance 0.06)        approved=False code=TABLE_CLEARANCE
```

The old row reproduces the left-over veto; the strict row proves the table
check itself fires. In the same mock the table collision object was confirmed
in the planning scene:

```text
id table_surface frame base_link dims [1.0, 1.0, 0.05]
object pose center [-0.001054, 0.006445, -0.102074]
object pose quat   [0.031523, 0.005155, 0.0, 0.99949]   (~3.6 deg tilt)
```

Evidence and the exact E2E command are in
[`docs/phase2/FIDUCIAL_GRASP_BRIDGE.md`](FIDUCIAL_GRASP_BRIDGE.md),
[`evidence/phase2/session3_20261117/`](../../evidence/phase2/session3_20261117/)
and
[`evidence/phase2/onsite_20261118/table_envelope_20261118.md`](../../evidence/phase2/onsite_20261118/table_envelope_20261118.md).

## Running

```bash
ros2 launch armpi_safety safety_supervisor.launch.py
ros2 service call /armpi_safety/validate_trajectory \
  arm_interfaces/srv/ValidateTrajectory "{trajectory: {joint_trajectory: {joint_names: [joint1,joint2,joint3,joint4,joint5], points: []}}}"
```

The empty trajectory is rejected (`TRAJECTORY_INVALID`), which is the expected
fail-closed behaviour. For a real check, pass a planned trajectory from MoveIt.

## Limitations

- **Partial hardware validation.** The veto has run on real MoveIt plans
  (2026-11-18/19), and the power cut was measured (the arm falls; the vendor
  stack recovers in ~89 s). All five joints were formally surveyed on
  2026-11-19 (`mapping_verified=true` for the operating envelope), backlash is
  now measured (1.117 mm), and no dedicated safety test has been done.
- **No e-stop integration.** The node cannot cut power; it can only publish a
  stop flag and reject plans. A future executor must treat the flag as
  authoritative.
- **No execution forwarding.** The layer does not sit between MoveIt and the
  controller, so a client that bypasses it is not protected.
- **No real-time guarantee.** Python/rclpy timing is not a safety PLC.
- **TCP-only envelope.** The workspace and table checks only see
  `end_effector_link`; they do not see the elbow, wrist or gripper body. The
  table collision object covers the missing geometry only when it is published
  into the MoveIt planning scene; the safety service itself has no environment
  collision check and no self-collision check beyond MoveIt's plan.
- **Static table plane.** The plane is a measured constant in `base_link`; it
  is invalid after the chassis moves. Re-measure/re-localize the table before
  table work at a new base pose.
- **Provisional velocity/acceleration values.** They must be revisited after the
  joint-limit survey and a supervised low-speed test.
- **State semantics.** `/joint_states` on the real robot is a cached command;
  the bus-servo `get_state` service is the hardware read. The watchdog treats
  the broadcast stream as a communication heartbeat, not as independent
  feedback.
