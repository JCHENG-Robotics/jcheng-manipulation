# Joint limits and mapping verification

> Status: **gripper measured 2026-11-18; torque-off survey UNBLOCKED and
> PARTIALLY run 2026-11-18; joints 3–5 still unmeasured.** The vendor
> `SetBusServoState` path works — its torque boolean is simply inverted on
> this servo firmware (`[1, 1]` / 0x0B releases, `[1, 0]` / 0x0C engages).
> joint2 was measured (322…873 pulse); joint1's positive stop reached 1164,
> outside the servo command range, and a command at 1164 was clamped to
> ~952. A vendor-node crash during joint3 stopped the run. The hand-found
> mechanical stops are wider than the servo EEPROM position limits on
> several joints, so the EEPROM limits are the effective commandable range;
> the provisional ±2.094 rad range is wider than the hardware. Hardware
> motion remains disabled and the backend keeps `mapping_verified=false`.

## What the 2026-11-18 survey established

The survey required a stand or second person because joints 2–5 carry the
arm against gravity. The gripper does not carry the arm and was measured
with a powered, no-object open/close test.

Full record: [`step6_torque_mapping.md`](../../evidence/phase2/onsite_20261118/step6_torque_mapping.md)
and [`step6_survey_partial.md`](../../evidence/phase2/onsite_20261118/step6_survey_partial.md).

| Joint | Result | State |
|---|---|---|
| joint1 | first run 501…503 (no movement); second run positive side **1164**, guard aborted; hold clamped to **952…954** | partial |
| joint2 | **322…873** (−0.746…1.562 rad, span 2.308), spread 1 pulse | measured |
| joint3 | pushed to **−41**, vendor node died | not measured |
| joint4 | — | not measured |
| joint5 | — | not measured |

Effective commandable limits from the read-only EEPROM table
(`rad = (pulse - 500) * 0.00418879`, joint3 inverted):

| Joint | EEPROM pulse | Effective rad |
|---|---|---|
| joint1 | 50…950 | −1.885…1.885 |
| joint2 | 160…830 | −1.424…1.382 |
| joint3 | 70…950 | −1.885…1.801 |
| joint4 | 100…900 | −1.676…1.676 |
| joint5 | 100…890 | −1.676…1.634 |

These are **not adopted** yet. joint2's low mechanical stop (322) is inside
its EEPROM range, so there the mechanical stop binds; the high side is
clamped by the EEPROM limit. Stop the hand motion at the first mechanical
resistance and never force a joint past the 0…1000 register range: doing
so killed the vendor node once. The tool now samples one servo per request
and aborts before the next read when a pulse leaves 0…1000.

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

### Gripper (2026-11-18, measured)

| Item | Result |
|---|---|
| Sign | Increasing `r_joint` closes; `l_joint` mirrors |
| Open stop | `r_joint = -0.8378`; servo register 502 |
| Closed stop | `r_joint = 0.0`; servo register 642 |
| Range | 0.8378 rad |
| Evidence | [`evidence/phase2/onsite_20261118/`](../../evidence/phase2/onsite_20261118/) |

The register is the linkage shaft, not the joint angle; `/joint_states` reports
the commanded joint value, so only the bus-servo read shows contact/stall.
`open = -0.80` in the SRDF keeps ~20 counts of margin from the open stop;
`close = 0.0` is the mechanical limit and stalls the empty gripper.

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
| `survey` | Releases torque on one joint at a time, records both hand-found stops, restores torque after each joint. Waits for the DDS subscription, samples one servo per request, and aborts before the next read if a pulse leaves 0…1000. Requires `--i-have-support` and a TTY. |
| `restore` | Re-engages torque on all five arm servos with a position hold. |

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
   Ctrl-C). **Stop at the first mechanical resistance.** A joint forced past
   the 0…1000 register range killed the vendor node on 2026-11-18; the tool
   now aborts before the next read, but prevention is the operator's job.
4. Check the printed `repeat spread` and `inside provisional` values. Repeat a
   joint if the spread is greater than 5 pulses.
5. If anything feels wrong, stop the tool and run `--mode restore`. If the
   vendor node died, keep supporting the arm, restart only
   `ros_robot_controller` (not the container), then run `--mode restore`.

The tool publishes `SetBusServoState` only for torque (`[1, 1]` releases,
`[1, 0]` engages on this firmware) and a `ServosPosition` hold at the
current registers for recovery; it never commands a new joint position.

## Adopting measured limits (separate reviewed change)

Do not set `mapping_verified` from this document alone. After a physical
survey:

1. Compare every measured stop with the servo EEPROM position limit. If the
   mechanical stop is outside the EEPROM range, the EEPROM limit binds for
   commanded motion and the measured stop only proves the joint can be
   back-driven beyond it.
2. Convert each stop with the vendor convention (`joint3` is inverted):
   `rad = (pulse - 500) * 0.00418879` for joint1/2/4/5 and
   `rad = (500 - pulse) * 0.00418879` for joint3.
3. Update `armpi-debug-studio/configs/joint_mapping.yaml` with measured
   `min_raw`/`max_raw` and a new `mapping_version`; keep `verified: false`
   until the full set is reviewed. Keep `mapping_verified=false` while any
   joint lacks a measurement.
4. Add the survey JSON to `evidence/phase2/`, update this document and the
   calibration/URDF comments, rebuild the backend, and restart the service.
5. Only then consider a supervised single-joint motion; Board approval is still
   required before any hardware motion.

## Limitations

- The bus-servo position is the servo's own register, not an independent
  encoder. It can report outside 0…1000 when a joint is back-driven, but the
  servo only accepts commands inside its EEPROM position limit.
- A hand-found stop is only as repeatable as the operator and the mechanism;
  the tool records the spread but cannot eliminate it.
- Backlash and compliance mean the stop can differ depending on approach
  direction. Measure both directions if that matters.
- `joint3` has a one-sided provisional range (-2.094…0 rad); do not assume a
  symmetric ±2.09 range.
- The 2026-11-18 run measured joint2 and part of joint1 only; joints 3–5 are
  unmeasured, and the vendor node crashed once when joint3 was back-driven
  past its register range.
- The gripper (`r_joint`, servo 1) was measured on 2026-11-18 (see above); the
  arm joints remain partial.
