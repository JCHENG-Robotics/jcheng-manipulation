# Step 6 — arm joint-limit survey: BLOCKED by torque-control defect

> **Superseded later on 2026-11-18.** The torque path was not broken: the
> vendor SDK's boolean is inverted on this firmware (`[1, 1]` / 0x0B
> releases torque, `[1, 0]` / 0x0C engages). The survey was unblocked and
> partially run; see [step6_torque_mapping.md](step6_torque_mapping.md) and
> [step6_survey_partial.md](step6_survey_partial.md). This file remains the
> record of the failed attempts and the read-only EEPROM table.

Date: 2026-11-18 09:53–10:01 (robot local time, UTC+8). Operator and support
person present; arm supported throughout.

## Outcome

**The hand-found mechanical survey was NOT completed.** The vendor
`/ros_robot_controller/bus_servo/set_state` (`SetBusServoState`) path does not
apply torque changes on this stack, so a joint cannot be selectively freed and
re-engaged safely. The arm was recovered to torque-on after each attempt and is
stable at the pose below.

## Read-only data captured (safe)

Battery: **7704 mV**. All five arm servos torque-enabled.

| Joint | Servo | Position (pulse) | Joint (rad) | Torque | Provisional range (rad) |
|---|---|---:|---:|---:|---|
| joint1 | 6 | 503 | +0.0126 | 1 | −2.094 … 2.094 |
| joint2 | 5 | 594 | +0.3937 | 1 | −2.094 … 2.094 |
| joint3 | 4 | 955 | **−1.9059** | 1 | −2.094 … 0.0 |
| joint4 | 3 | 329 | −0.7163 | 1 | −2.094 … 2.094 |
| joint5 | 2 | 503 | +0.0126 | 1 | −2.094 … 2.094 |

`joint3` is lower than the `camera_forward` pose (−1.257 rad) because it sagged
during the first failed attempt; it is still inside the provisional range.

## Servo EEPROM limits and offsets (read-only, `[V]`)

| Servo | Joint | Position limit (pulse) | Offset |
|---|---|---|---:|
| 1 | r_joint | 150 … 700 | 115 |
| 2 | joint5 | 100 … 890 | −8 |
| 3 | joint4 | 100 … 900 | 13 |
| 4 | joint3 | 70 … 950 | 4 |
| 5 | joint2 | 160 … 830 | −1 |
| 6 | joint1 | 50 … 950 | −7 |

These are **firmware/EEPROM travel bounds**, not hand-measured mechanical
stops. They are wider than the vendor software ranges in several places and
must not be reported as the measured joint limits.

## Failure sequence

### Attempt 1 — original survey tool

- Command: `--mode survey --i-have-support`.
- At `joint1` / `ready`, the tool disabled servo 6, then read only that servo
  every 0.1 s and timed out: `servo 6 torque did not become 0`.
- Its outer `restore_all_torque()` used the `SetBusServoState` 0x0B enable for
  all five arm servos. This did not re-engage them.
- Result: **all five arm servos torque-off**; `joint3` (servo 4) sagged to
  pulse 1038, outside the nominal 0…1000 range. The arm was supported.
- Recovery: `tools/phase2/servo_hold.py`, which publishes the current positions
  as a slow `ServosPosition` command. Torque returned to 1 on all six servos;
  servo 4 settled at 960.

### Attempt 2 — fixed selective-disable tool

- Fixes: publish disable, wait 1.5 s **without service calls**, read once;
  reject collateral torque-off; restore by position hold.
- Preflight position hold passed (`preflight OK`, all torque 1).
- At `joint1` / `ready`, the disable message again had no effect:
  `servo 6 torque did not become 0 (last state: all 1)`.
- The tool aborted and restored by position hold; the arm stayed torqued.

### Attempt 3 — gripper probe (safe, no arm motion)

- `torque_probe.py` published the same `SetBusServoState` disable for servo 1
  (empty gripper), waited 2 s without service calls, then read.
- Torque stayed **1**; the hold command left it at 1.
- This isolates the failure to the state topic, not the arm or the survey
  sequencing.

## Diagnosis

- `/ros_robot_controller/bus_servo/set_state` has **one subscriber**
  (`ros_robot_controller`), type `ros_robot_controller_msgs/msg/SetBusServoState`,
  RELIABLE/KEEP_LAST(10). The vendor node is alive (`/battery` at 10.04 Hz) and
  runs CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).
- The dedicated position topic
  `/ros_robot_controller/bus_servo/set_position` **does** work: a
  `ServosPosition` hold at the current positions re-engages all servos.
- Waiting without `get_state` calls did not help, so single-threaded callback
  starvation is not the only cause.
- The recurring Fast DDS `serdata.cpp:384` (`string data is not null-terminated`
  / `invalid data size`) errors correlate with the state-topic traffic; the
  message may be failing deserialization or being dropped in the vendor node.
  Root cause unresolved.
- The 0x0B enable command is also unreliable; **position hold is the only
  verified recovery path**.

## Safety conclusions

1. **Do not run `joint_limit_survey.py --mode survey`** until the
   `SetBusServoState` callback is fixed and a selective disable has been
   demonstrated on one joint.
2. Mechanical limits remain **provisional**. The arm should not be used for
   full-range, continuous or autonomous motion.
3. The arm is currently stable, torque-on, inside the provisional ranges.
4. Any future survey must have support, the position-hold recovery ready, and
   a preflight that verifies all servos can be re-engaged.

## Next steps

- Instrument the vendor `ros_robot_controller` node to log entry into
  `set_bus_servo_state` and check whether the callback runs.
- Compare the installed `ros_robot_controller_msgs` definitions between the
  vendor build and the Python tool; test a minimal `SetBusServoState` message.
- If the ROS path cannot be fixed, stop `ros_robot_controller`, use the vendor
  SDK directly on `/dev/rrc`, then restart the stack. That is a separate,
  reviewed intervention.
- Only then re-run a single-joint survey, followed by the full set.
