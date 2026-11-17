# Conservative Safety Layer

> Status: **Implemented and mock-validated (2026-11-17, Session 3).** The layer
> is a state watchdog and trajectory **veto**, not a controller or an e-stop.
> Joint limits remain provisional and no hardware safety test has been done.

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
| State limits | positions inside provisional joint ranges | `STATE_INVALID` / `JOINT_LIMIT` |
| Trajectory shape | arm joints present, positions length matches, ≤ 5000 points | `TRAJECTORY_INVALID` |
| Timestamps | strictly increasing | `TIMESTAMP` |
| Duration | ≤ 30 s | `DURATION` |
| Position limits | every waypoint in range (1e-3 rad tolerance) | `JOINT_LIMIT` |
| Velocities | explicit, else finite differences; ≤ 1.2 rad/s | `VELOCITY_LIMIT` |
| Accelerations | explicit, else finite differences; ≤ 0.5 rad/s² | `ACCELERATION_LIMIT` |
| Start state | first point within 0.08 rad of the latest state | `START_STATE` |
| Workspace | `end_effector_link` TCP inside the `base_link` box on every waypoint | `WORKSPACE` |
| FK availability | fail closed when workspace FK is unavailable | `WORKSPACE` |

A **real state fault latches** the stop: once stale/out-of-range/non-finite
state is seen, `safety_stop` stays true until `reset_stop` succeeds and the
current state is valid. A rejected **plan** is a veto, not a robot fault; it does
not latch the stop.

## Configuration

[`ros2_ws/src/armpi_safety/config/safety_limits.yaml`](../../ros2_ws/src/armpi_safety/config/safety_limits.yaml)
holds the provisional limits. The main values:

```yaml
joint_limits:
  joint3: {lower: -2.09, upper: 0.0, max_velocity: 1.2, max_acceleration: 0.5}
state: {timeout_s: 0.5}
start_state: {max_distance_rad: 0.08}
trajectory: {max_duration_s: 30.0, max_points: 5000}
workspace:
  frame: base_link
  enabled: true
  required: true
  lower: [-0.05, -0.30, 0.00]
  upper: [0.45, 0.30, 0.55]
```

The position bounds are the intersection of the vendor software mapping and the
URDF; they are **not measured**. Velocity and acceleration are deliberately
below the URDF's `10` values, but the planner also uses 0.1 scaling, so this is
a first conservative envelope rather than a tuned servo limit.

## Mock verification

Unit tests: `armpi_safety/test/test_policy.py` covers a valid trajectory,
stale state, joint/velocity/acceleration limits, start-state mismatch,
workspace excursion, missing FK, missing joint, non-monotonic timestamps,
duration, reset-stop semantics and non-finite positions.

Service test on a real planned trajectory (mock MoveIt, plan-only):

```text
SAFETY_CASE valid      approved=True  code=OK
SAFETY_CASE joint_limit approved=False code=JOINT_LIMIT
SAFETY_CASE velocity   approved=False code=VELOCITY_LIMIT
SAFETY_CASE timestamp  approved=False code=TIMESTAMP
RESULT PASS
```

Evidence and the exact E2E command are in
[`docs/phase2/FIDUCIAL_GRASP_BRIDGE.md`](FIDUCIAL_GRASP_BRIDGE.md) and
[`evidence/phase2/session3_20261117/`](../../evidence/phase2/session3_20261117/).

## Running

```bash
ros2 launch armpi_safety safety_supervisor.launch.py
ros2 service call /armpi_safety/validate_trajectory \
  arm_interfaces/srv/ValidateTrajectory "{trajectory: {joint_trajectory: {joint_names: [joint1,joint2,joint3,joint4,joint5], points: []}}}"
```

The empty trajectory is rejected (`TRAJECTORY_INVALID`), which is the expected
fail-closed behaviour. For a real check, pass a planned trajectory from MoveIt.

## Limitations

- **No hardware validation.** Mechanical limits, servo response, backlash and
  the physical stop/power cut are unmeasured.
- **No e-stop integration.** The node cannot cut power; it can only publish a
  stop flag and reject plans. A future executor must treat the flag as
  authoritative.
- **No execution forwarding.** The layer does not sit between MoveIt and the
  controller, so a client that bypasses it is not protected.
- **No real-time guarantee.** Python/rclpy timing is not a safety PLC.
- **Workspace only.** There is no environment collision check, no self-collision
  check beyond MoveIt's plan, and no velocity smoothing.
- **Provisional velocity/acceleration values.** They must be revisited after the
  joint-limit survey and a supervised low-speed test.
- **State semantics.** `/joint_states` on the real robot is a cached command;
  the bus-servo `get_state` service is the hardware read. The watchdog treats
  the broadcast stream as a communication heartbeat, not as independent
  feedback.
