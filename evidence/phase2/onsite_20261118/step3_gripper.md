# Step 3 — gripper sign and range (no object)

Date: 2026-11-18 09:24–09:40 (robot local time, UTC+8). Supervisor present,
gripper empty, main power switch within reach.

## Method

- Trajectories sent to `/gripper_controller/follow_joint_trajectory`
  (`control_msgs/action/FollowJointTrajectory`).
- Each goal started at the current `r_joint` and moved in small steps.
- Physical position read from
  `/ros_robot_controller/bus_servo/get_state`
  (`ros_robot_controller_msgs/srv/GetBusServoState`, servo ID 1,
  `get_position=1`, `get_torque_state=1`), after settling.
- `/joint_states` was also recorded, but it echoes the commanded joint value.

Vendor mapping for `r_joint`: servo ID 1, `init=700`, `min=0`, `max=1000`,
not flipped. Command pulse = `700 + r_joint / 0.00418879`; joint value =
`(pulse - 700) * 0.00418879`.

## Sequence

| Command `r_joint` (rad) | Command pulse | Servo register | Torque | Operator observation |
|---:|---:|---:|---:|---|
| `-0.837758` (start) | 500 | — | 1 | open |
| `-0.737758` | 524 | — | — | **fingers close** (sign confirmed) |
| `0.0` | 700 | **642** | 1 | fully closed; servo held at 642 |
| `-0.837758` | 500 | 516 | 1 | reopened |
| `-0.86` | 494 | 508 | 1 | still open; small movement |
| `-0.88` | 490 | 506 | 1 | at open stop; no further movement |
| `-0.837758` | 500 | 502 | 1 | open stop position |
| `-0.243` | 642 | 622 | 1 | partially closed |
| `-0.80` (final) | 509 | 523 | 1 | safe open, clear of stop |

The `0.0` close was repeated from `-0.837758` and again reached the closed
state; the `-0.243` read was taken after settling and remained 622 across three
consecutive reads.

## Conclusions

- **Sign:** increasing `r_joint` closes the fingers. `l_joint` mirrors with
  multiplier `-1`.
- **Mechanical range:** `r_joint = -0.8378` (open stop) … `0.0` (closed stop),
  span **0.8378 rad**.
- **Open stop:** commanding `-0.86` / `-0.88` moved the register only
  508 → 506, and `-0.8378` settles at 502. The stop is at/near raw 500.
- **Closed stop:** commanding `0.0` (raw 700) leaves the register at **642**
  with the fingers fully closed. The servo is stalled against the mechanism;
  do not hold this pose with an empty gripper.
- **Register vs joint:** the servo register (502–642) is the linkage shaft, not
  the finger joint. `/joint_states` reported `0.0` while the register was 642,
  so it cannot be used to detect contact or stall.

## Adopted model values

- URDF / MoveIt `r_joint` limits: **`-0.8378 … 0.0`**.
- SRDF `open`: **`-0.80`** (about 20 counts inside the open stop).
- SRDF `close`: **`0.0`** (mechanical closed limit; stalls when empty).
- Simulation rest pose: `r_joint = -0.80`.

## Caveats

- The stops were found by commanding the servo into the mechanism, not by
  hand-turning a torque-disabled joint. The recorded register values are the
  servo's own position, not external metrology.
- The gripper has no force/contact feedback. Grasping must use small, slow
  commands and must not rely on `/joint_states` to know when the fingers touch.
- No object was present. Gripper tolerance-envelope trials remain a Phase 3
  measurement.
