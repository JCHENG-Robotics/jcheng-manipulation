# Forward-facing lowest reach — 2026-11-19

> Status: **measured at the mechanical stops.** The arm was placed on the
> floor (not the table). Facing forward (`joint1 = 0`) the true mechanical
> maximum is `joint2 = 322` and `joint3 = 939` pulse. The measured TCP is
> `(0.191, −0.002, −0.0565) m`, tool down. The operator's ruler reads
> **10 mm from the floor to the lowest point of the gripper** and **15 mm to
> the nominal TCP**. The mechanism has no more forward/down travel: joint2
> is on its hand-measured stop (322) and joint3 stops at 939, short of its
> EEPROM limit 950. After the measurement the arm was returned to the
> adopted-low forward pose; the actual registers are `496, 339, 925, 334,
> 500` and `/joint_states` reports `496, 342, 929, 339, 500` (the known
> 3–5 pulse command-vs-register offset).

## Setup

- Robot on the floor; battery **8268 mV**.
- Direction: **forward**, `joint1 = 0`. The first attempt was made at
  `joint1 = −1.5709` (side); the operator rejected it and asked for a
  forward-facing descent. The side pose reached the same TCP z (−0.0455 m)
  but with 40.9 mm model clearance because the measured plane is lower at
  `y = −0.13`; it is kept only as a cross-check.
- Commanded motion used the real MoveIt path for the adopted-limit pose and
  the direct `ServosPosition` path (`tools/phase2/pulse_move.py`) for the
  last pulses beyond the adopted limits. Torque was never released.
- Measured positions are the **bus-servo `get_state` registers**, not
  `/controller_manager/servo_states`; that topic echoes the command and
  differed by 3–5 pulses while holding (consistent with
  `docs/hardware/measured_joint_limits.md`).

## 1. Adopted-limit forward pose (MoveIt, safety approved)

Target `joint1 0, joint2 −0.6618, joint3 −1.8012, joint4 −0.6704,
joint5 0`; velocity/acceleration scaling 0.05.

| Item | Value |
|---|---|
| Plan | 144 waypoints, 14.263 s, `error_code=1` |
| Safety | `approved=True code=OK` (TCP table-plane check + table collision object) |
| Execute | `execute_status=4` (SUCCEEDED), `execute_error_code=1` |
| Final plan joints | `0.0007, −0.6611, −1.8012, −0.6720, 0.0018` |
| FK TCP / clearance | `(0.196, 0.000, −0.0444) m`, **35.0 mm** above the fitted plane |
| Minimum along the path | 35.0 mm (the endpoint) |

The TCP is the vendor nominal grasp point (`end_effector_link`, 80 mm past
`link5`). The path was collision-checked with the table/floor object; it did
not go below the endpoint.

## 2. Pulse steps beyond the adopted limits

`pulse_move.py` publishes one `ServosPosition` per step (3.0 s, 2.0 s),
reads the registers back, and stops when a commanded joint does not follow.
All positions stayed positive (a negative register read kills the vendor
node). `joint3` is inverted, so a larger pulse is a lower joint angle.

| Step | Joint2 | Joint3 | Joint4 | Measured after (j2/j3/j4) | Result |
|---|---:|---:|---:|---|---|
| adopted | 342 | 929 | 339 | 342/929/339 (command) | MoveIt pose |
| 1 | 337 | 934 | 349 | 337/931/345 | arm tracked; gripper stall only |
| 2 | 332 | 939 | 359 | 329/937/354 | `TRACK_OK` |
| 3 | 327 | 944 | 369 | 328/939/365 | joint3 stops at 939; joint2 barely moves |
| 4 | 325 | 941 | 368 | 325/939/366 | joint2 reaches 325; joint3 still 939 |
| 5 | **322** | 939 | 366 | **322/939/366** | `TRACK_OK`; joint2 on its measured stop |

Step 3 was the first mechanical resistance: joint3 had reached 939 and
joint2 moved only 1 pulse. Step 5 confirms joint2 can still reach 322
exactly; joint3 never moves past 939. **The EEPROM 950 of joint3 is not
reachable mechanically.** Raw records: [pulse_steps_20261119.yaml](pulse_steps_20261119.yaml).

## 3. Final pose and physical measurement

| Joint | Register | Joint (rad) |
|---|---:|---:|
| joint1 | 496 | −0.0168 |
| joint2 | **322** | −0.7456 |
| joint3 | **939** | −1.8389 |
| joint4 | 366 | −0.5613 |
| joint5 | 501 | +0.0042 |

- FK TCP `(0.1907, −0.0018, −0.0565) m`, tool `z·z = −0.99996`, model
  clearance **23.0 mm** above the fitted plane (`z = −0.0794` at that xy).
- Operator measurement at this pose: **10 mm lowest point**, **15 mm TCP**.
  The fingertip is therefore ~5 mm below the nominal TCP, and the fitted
  plane is ~8 mm below the physical floor at this point.
- Adopted-limit pose physical estimate: 35.0 mm model − 8 mm plane bias
  ≈ **27 mm TCP**, ≈ **22 mm lowest point**.

## Findings

1. **The true forward-facing lowest point is 10 mm above the floor** at
   `joint2 = 322`, `joint3 = 939`; the arm cannot go lower. The nominal TCP
   is 15 mm above the floor.
2. **joint3's mechanical stop is 939 pulse**, not the EEPROM limit 950.
   The adopted 930 is 9 pulses (≈2.3 mm at the TCP) inside the stop.
3. **joint2's stop 322 is commandable**: the servo reached and held 322
   without a stall, so the adopted 342 is 20 pulses (≈5 mm) inside the stop.
4. **The fitted table/floor plane is ~8 mm below the physical floor** at the
   measurement point, and the fingertip is ~5 mm below the nominal TCP. The
   safety `clearance_m: 0.01` is measured against the fitted plane at the
   TCP; at the mechanical maximum it corresponds to only ~2 mm of physical
   fingertip clearance. Do not relax the adopted limits, and correct the
   plane/fingertip offsets before using the 10 mm clearance as a physical
   guarantee.
5. `/controller_manager/servo_states` is a **command echo**; the
   `get_state` service is the register read. Use the service for metrology.
   A partial `ros_robot_controller` restart can additionally leave
   `/joint_states` **stale** (see the incident above); reconcile the two
   before planning.

## Incidents

### Clock: MoveIt refused to execute twice

The robot has no working RTC (`RTC time 1970-01-01`) and no NTP; it was
**12 h 26 min behind** the laptop. MoveIt's `current_state_monitor` rejected
the `/joint_states` stream as stale and ABORTED before any motion:

```text
latest received state has time 1795005216.840442
requested time 1795050034.459578
Failed to validate trajectory: couldn't receive full current joint state within 1s
```

The robot clock was set to the laptop UTC and then given a **+2 s lead**
(the remaining offset was ~0.37 s, enough to fail the "state at or after the
request time" check). Execution then succeeded. This is required for every
cross-host MoveIt execute; NTP is not available.

### Vendor service died during the back-off

After the mechanical-max measurement the arm was commanded back to
`342/929/339`. The `ros_robot_controller` bus-read thread stopped after
reading `joint4 = 343`: the `get_state` service stopped responding
(timeouts for all six servos) while the process stayed alive and the
`servo_states` topic continued. `kill -9` plus starting only
`ros_robot_controller` (not the container) restored the service, but the
restart moved the arm to the known post-restart pose
`joint1=496, joint2=595, joint3=833, joint4=113, joint5=500`
(TCP `z = +0.111 m`, 190 mm above the floor). **The back-off was not
completed; the restart itself is a motion.** Do not restart the node with
the arm near the floor; support it or move it clear first.

### A partial restart leaves `/joint_states` stale

After that restart, `/joint_states` still reported the **pre-restart
command pose** (`joint1 −0.0042, joint2 −0.6618, joint3 −1.7970,
joint4 −0.6744`) while the actual servos were at `595/833/113`. The safety
node therefore reported `valid: true, stop: false` for a pose the arm was
not in. The actual arm was moved back to the reported pose with
`pulse_move.py` (4 s; the linear pulse path was checked offline, minimum
model clearance 35.1 mm at the endpoint). Final actual registers:
`joint1=496, joint2=339, joint3=925, joint4=334, joint5=500`; reported
`496/342/929/339/500`, the known 3–5 pulse offset.

**After restarting only `ros_robot_controller`, read the service registers
and reconcile them with `/joint_states` before any planning.** Restarting
the full vendor stack would also refresh `/joint_states`, but it runs the
vendor init and moves the arm, so it must not be done near the floor.

## Files

- [adopted_low_plan_20261119.json](adopted_low_plan_20261119.json) — MoveIt plan
- [adopted_low_exec_20261119.json](adopted_low_exec_20261119.json) — executed plan
- [pulse_steps_20261119.yaml](pulse_steps_20261119.yaml) — raw step records
- `tools/phase2/pulse_move.py` — supervised raw-pulse step tool

## What was not done

- No change to `joint_mapping.yaml`, URDF, MoveIt or safety limits. The
  adopted 20-pulse / EEPROM-bounded limits remain; `mapping_verified` stays
  false.
- No hand-push torque-off survey in this session.
- The floor plane was not re-measured; the 8 mm plane bias is inferred from
  the two ruler readings, not a new depth-plane fit.
