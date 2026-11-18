# Step 6 — torque-command semantics found (blocked survey unblocked)

Date: 2026-11-18, after the blocked survey attempt. Operator at the robot;
empty gripper; arm supported and stable.

## Conclusion

**The vendor SDK's torque boolean is inverted on this servo firmware.**
`SetBusServoState` is not broken; the register read is truthful.

| Message field | SDK meaning | Actual effect on this robot |
|---|---|---|
| `enable_torque=[1, 1]` | enable (0x0B) | **releases** torque; register reads 0 |
| `enable_torque=[1, 0]` | disable (0x0C) | **engages** torque; register reads 1 |

A selective release has now been demonstrated on one servo (gripper,
servo 1) with the five arm servos left at torque 1.

## Evidence sequence

1. `joint_limit_survey.py`-style single publish of a small gripper move
   was lost to a DDS discovery race: the publisher process had just
   started, so the sample was dropped before the subscription matched.
2. `ros2 topic pub` (which waits for discovery) on
   `/ros_robot_controller/bus_servo/set_state` with
   `present_id=[1, 1], position=[1, 540]` moved the gripper register
   **522 → 525** (repeated samples restart the servo interpolation, so it
   only moved part of the way). Restore to 522 was issued.
3. `enable_torque=[1, 1]` (SDK "enable", 0x0B), 10 samples at 5 Hz:

   ```
   servo 1: position=525  enable_torque=[0]
   servos 2..6: enable_torque=[1]  (unchanged)
   ```

   Operator: gripper **free to move by hand** ("能动"). Selective release
   demonstrated.
4. Operator moved the gripper by hand: register **525 → 579** (further
   proof that torque was released).
5. `enable_torque=[1, 0]` (SDK "disable", 0x0C), 10 samples at 5 Hz:

   ```
   servo 1: position=579  enable_torque=[1]
   ```

   Torque engaged at the new position.

## Commands

> Note: [`torque_probe_20261118.json`](torque_probe_20261118.json) is the
> probe run from step 1. Its `callback_not_running: true` is a **false
> negative** caused by the DDS discovery race; the CLI test in step 2 and the
> torque tests above are the authoritative result. The probe now waits for
> the subscription before publishing.

Published with the vendor CycloneDDS environment
(`source ~/.zshrc` inside `ArmPiUltra`):

```bash
ros2 topic pub -r 5 --times 10 /ros_robot_controller/bus_servo/set_state \
  ros_robot_controller_msgs/msg/SetBusServoState \
  "{state: [{present_id: [1, 1], enable_torque: [1, 1]}], duration: 0.0}"

ros2 topic pub -r 5 --times 10 /ros_robot_controller/bus_servo/set_state \
  ros_robot_controller_msgs/msg/SetBusServoState \
  "{state: [{present_id: [1, 1], enable_torque: [1, 0]}], duration: 0.0}"
```

State read:

```bash
ros2 service call /ros_robot_controller/bus_servo/get_state \
  ros_robot_controller_msgs/srv/GetBusServoState \
  "{cmd: [{id: 1, get_position: 1, get_torque_state: 1}]}"
```

## Consequence for the survey tool

`joint_limit_survey.py` now publishes `[1, 1]` to release and `[1, 0]` to
engage, waits for the subscriber count to become non-zero before the first
publish, and keeps the position hold as the recovery fallback. The
supplementary `torque_probe.py` is kept as a re-verification tool.

This also explains the 2026-11-18 blocked survey: its "restore" path sent
`[1, 1]` (0x0B) to all five arm servos, which by this mapping **released
all five** instead of re-engaging them.

## Remaining safety rules

- The arm joint survey still requires a stand or second person; joints 2–5
  fall when released.
- No electronic e-stop; the power switch must stay in reach.
- Mechanical limits are not yet measured; do not treat this file as a
  limit measurement.
