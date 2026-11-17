# Joint limits and mapping verification

> Status: **read-only servo state captured on 2026-11-17; mechanical limits NOT
> measured.** The bus-servo positions and torque states were read on the real
> robot, but no joint was moved by hand and no torque was disabled. Joint limits
> remain **provisional**, hardware motion remains disabled, and the backend
> keeps `mapping_verified=false`.

## Why the physical survey is pending

Measuring a mechanical limit means disabling torque on one joint and moving it
by hand to both stops. Joints 2–5 carry the arm against gravity, so the arm can
fall as soon as torque is released. The survey was deferred until the robot has
a stand or a second person supporting the arm.

## What was verified (read-only)

Evidence: [`evidence/phase2/joint_limits_20261117/`](../../evidence/phase2/joint_limits_20261117/).

| Item | Value |
|---|---|
| Robot / container | Pi 5, `ArmPiUltra` (vendor stack running) |
| Battery | 7424 mV |
| Tool | `tools/phase2/joint_limit_survey.py --mode read --servo-reads 5` |
| Tool SHA-256 | `336c85332d864f83aa22f5e60b021940ca7980863f684069273a78697784db93` |
| Torque state | all five arm servos `1` (enabled) before and after the read |

| Joint | Servo ID | Pulse | Joint (rad) | Provisional range (rad) |
|---|---|---|---|---|
| joint1 | 6 | 497 | -0.013 | -2.094 … 2.094 |
| joint2 | 5 | 642 | 0.595 | -2.094 … 2.094 |
| joint3 | 4 | 808 | -1.290 | -2.094 … 0.000 |
| joint4 | 3 | 331 | -0.708 | -2.094 … 2.094 |
| joint5 | 2 | 500 | 0.000 | -2.094 … 2.094 |

The pose is consistent with the controller's command echo (`joint2=0.5948`,
`joint4=-0.7079`, `joint5=0`). `joint3` differs by ~0.033 rad (1.9°) and
`joint1` by ~0.013 rad, within the vendor's documented command-vs-bus-servo
scatter (up to 2.16°, RMS 1.13°). This is a **consistency check**, not a
mechanical-limit measurement.

The sign/offset convention is also supported indirectly by the hand-eye
calibration: 14 chessboard poses at ±0.06–0.12 rad produced 2.94 mm / 0.39°
board-pose scatter using these measured servo positions. A wrong joint sign or
servo ID would not fit that consistently. See
[`docs/phase2/CALIBRATION.md`](CALIBRATION.md). This does **not** verify the
mechanical travel.

## Important: use the vendor CycloneDDS environment

The vendor nodes run with:

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
```

A shell started with `docker exec ... /bin/bash -lc 'source /opt/ros/humble/setup.bash; ...'`
does **not** inherit these variables. Topics may still arrive across RMW
implementations, but **services do not respond**: `/ros_robot_controller/bus_servo/get_state`
and even `/controller_manager/get_parameters` time out. Always run the survey
through `zsh` with `source ~/.zshrc` (which sets both variables), or export
them explicitly.

## Tool

`tools/phase2/joint_limit_survey.py` has three modes:

| Mode | Effect |
|---|---|
| `read` (default) | Subscribes to the battery and reads position + torque registers. Sends nothing. |
| `survey` | Disables torque on one joint at a time, records both hand-found stops, restores torque after each joint. Requires `--i-have-support` and a TTY. |
| `restore` | Re-enables torque on all five arm servos. |

Copy it into the container and run it with the vendor shell:

```bash
# laptop -> robot
scp -i ~/.ssh/id_ed25519_armpi tools/phase2/joint_limit_survey.py \
  pi@<robot>:/home/pi/docker/tmp/
ssh -i ~/.ssh/id_ed25519_armpi pi@<robot> \
  'docker cp /home/pi/docker/tmp/joint_limit_survey.py ArmPiUltra:/tmp/'

# read-only check
ssh -i ~/.ssh/id_ed25519_armpi -t pi@<robot> \
  'docker exec -it -u ubuntu ArmPiUltra /bin/zsh -c \
     "source ~/.zshrc >/dev/null 2>&1; \
      python3 /tmp/joint_limit_survey.py --mode read"'
```

## Physical survey procedure (when a stand/second person is available)

1. Clear the workspace; support the arm so it cannot fall when a joint is free.
2. Run:

   ```bash
   ssh -i ~/.ssh/id_ed25519_armpi -t pi@<robot> \
     'docker exec -it -u ubuntu ArmPiUltra /bin/zsh -c \
        "source ~/.zshrc >/dev/null 2>&1; \
         python3 /tmp/joint_limit_survey.py --mode survey \
           --i-have-support --servo-reads 5 \
           --output /tmp/joint_limits_survey_<date>.json"'
   ```

3. For each joint, type `ready` only after it is supported, move it slowly to
   one stop, press Enter, move to the other stop, press Enter. Torque is
   restored immediately after the second reading (and in a `finally` block on
   Ctrl-C).
4. Check the printed `repeat spread` and `inside provisional` values. Repeat a
   joint if the spread is greater than 5 pulses.
5. If anything feels wrong, stop the tool and run `--mode restore`.

The tool never publishes a position command. It only publishes
`SetBusServoState` with `enable_torque`.

## Adopting measured limits (separate reviewed change)

Do not set `mapping_verified` from this document alone. After a physical
survey:

1. Confirm every measured stop is inside the 0…1000 pulse range and the span is
   plausible for that joint.
2. Convert each stop with the vendor convention (`joint3` is inverted):
   `rad = (pulse - 500) * 0.00418879` for joint1/2/4/5 and
   `rad = (500 - pulse) * 0.00418879` for joint3.
3. Update `armpi-debug-studio/configs/joint_mapping.yaml` with measured
   `min_raw`/`max_raw` and a new `mapping_version`; keep `verified: false`
   until the full set is reviewed.
4. Add the survey JSON to `evidence/phase2/`, update this document and the
   calibration/URDF comments, rebuild the backend, and restart the service.
5. Only then consider a supervised single-joint motion; Board approval is still
   required before any hardware motion.

## Limitations

- The bus-servo position is the servo's own register, not an independent
  encoder. There is no external metrology.
- A hand-found stop is only as repeatable as the operator and the mechanism;
  the tool records the spread but cannot eliminate it.
- Backlash and compliance mean the stop can differ depending on approach
  direction. Measure both directions if that matters.
- `joint3` has a one-sided provisional range (-2.094…0 rad); do not assume a
  symmetric ±2.09 range.
- The gripper (`r_joint`, servo 1) is excluded from G1 and was not read.
