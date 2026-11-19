# Joint limits and mapping verification

> Status: **formally surveyed on all five arm joints 2026-11-19; the mapping
> is `verified: true` for the operating envelope.** One joint at a time with
> torque released, in the vertical pose plus joint2 at the low operating pose:
> joint1 58…957, joint2 186…873 (upright) / **327…795 (low pose, hand)**,
> joint3 100…923, joint4 130…882, joint5 122…869 pulse, spreads 0–2 pulses.
> The adopted high bound of joint2 was **tightened from 810 to 775** because
> the low-pose hand stop is 795. The stops are **self-collision-limited and
> pose-dependent**, so the single static limits are verified for the operating
> envelope, not as universal mechanical stops. Evidence:
> [`evidence/phase2/onsite_20261119/joint_limits/`](../../evidence/phase2/onsite_20261119/joint_limits/README.md). A 2026-11-19 forward-facing powered probe
> confirmed joint2 **322** is commandable and measured joint3's mechanical
> stop at **939** (EEPROM 950 is unreachable); the lowest physical gripper
> point is **10 mm above the floor**. The adopted limits were not changed.
> See [`evidence/phase2/onsite_20261119/lowest_reach/`](../../evidence/phase2/onsite_20261119/lowest_reach/). The vendor `SetBusServoState` path works — its
> torque boolean is simply inverted on this servo firmware (`[1, 1]` / 0x0B
> releases, `[1, 0]` / 0x0C engages). joint2 was measured (322…873 pulse)
> and joint4's high stop reached 901; joint3 was observed by hand over
> 23…940 pulse; joint5 cannot be hand-measured because the released wrist
> falls under gravity. The hand-found mechanical stops are wider than the
> servo EEPROM position limits on several joints, so the EEPROM limits are
> the effective commandable range; the provisional ±2.094 rad range is
> wider than the hardware. **All four vendor-node crashes had one cause: a
> negative position register raises a `uint16` `AssertionError` inside the
> vendor `get_state` callback.** Hardware motion remains disabled and the
> backend keeps `mapping_verified=false`.

## What the 2026-11-18 surveys established

The survey required a stand or second person because joints 2–5 carry the
arm against gravity. The gripper does not carry the arm and was measured
with a powered, no-object open/close test.

Full records: [`step6_torque_mapping.md`](../../evidence/phase2/onsite_20261118/step6_torque_mapping.md),
[`step6_survey_partial.md`](../../evidence/phase2/onsite_20261118/step6_survey_partial.md)
and [`step6_survey_run2_20261118.md`](../../evidence/phase2/onsite_20261118/step6_survey_run2_20261118.md).

| Joint | Result | State |
|---|---|---|
| joint1 | first run 501…503 (no movement); second run positive side **1164**, guard aborted; hold clamped to **952…954** | high bound beyond the EEPROM limit; low side unmeasured |
| joint2 | **322…873** (−0.746…1.562 rad, span 2.308), spread 1 pulse; powered hold at **322** confirmed 2026-11-19 | measured; the low stop is inside the EEPROM range |
| joint3 | hand travel **23…940** pulse; a powered 2026-11-19 probe stops at **939** (EEPROM 950 unreachable) | low stop measured; positive side not commandable |
| joint4 | **high stop 901** (+1.680 rad), spread 1 pulse; low taken from the EEPROM limit | high measured; low unmeasured |
| joint5 | hand travel reached **875**; released at 258 it **fell to 62 in 0.75 s** (196 pulses), then a negative read killed the node | not measurable by hand; EEPROM bounds used |

The wrist fall is a mechanism property, not an operator error: a released
joint is not held against gravity, and an extended arm gives the wrist a
large moment. The final tool aborts when a released joint drops more than
**60 pulses between two 0.75 s reads** and restores torque at the current
position, which catches the fall before the next (possibly negative) read.

Effective commandable limits from the read-only EEPROM table
(`rad = (pulse - 500) * 0.00418879`, joint3 inverted):

| Joint | EEPROM pulse | Effective rad |
|---|---|---|
| joint1 | 50…950 | −1.885…1.885 |
| joint2 | 160…830 | −1.424…1.382 |
| joint3 | 70…950 | −1.885…1.801 |
| joint4 | 100…900 | −1.676…1.676 |
| joint5 | 100…890 | −1.676…1.634 |

The EEPROM limits bind wherever a hand stop is farther out; joint2's low
mechanical stop (322) is inside its range and binds there instead. **The
adopted limits are these bounds minus a 20-pulse margin** (see
[Adopted limits](#adopted-limits) below). Never force a joint past the
0…1000 register range: a negative register is not merely out of range, it
kills the vendor node during that read.

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

### Gripper (2026-11-18, re-measured 19:59)

| Item | Result |
|---|---|
| Sign | Increasing `r_joint` closes; `l_joint` mirrors |
| Open stop | servo register **167**, reached at command raw **~150** |
| Adopted open | `r_joint = -2.22` (raw 170, register 188), 21 counts inside the stop |
| Closed stop | `r_joint = 0.0` (raw 700); servo register **640** |
| Range | **2.22 rad** (command travel 170…700 = 530 raw counts) |
| Backlash | ~34 register counts (raw 400 reads 417 going open, 383 returning) |
| Evidence | [`evidence/phase2/onsite_20261118/gripper_stops_20261118/`](../../evidence/phase2/onsite_20261118/gripper_stops_20261118/) |

The register is the linkage shaft, not the joint angle; `/joint_states` reports
the commanded joint value, so only the bus-servo read shows contact/stall.
`open = -2.22` in the SRDF sits ~20 counts inside the open stop;
`close = 0.0` is the mechanical limit and stalls the empty gripper.

> Superseded: the first 2026-11-18 pass recorded an open stop of
> `r_joint = -0.8378` / register 502 with a 0.8378 rad range. That was a
> **binding stall**, not the stop — the fingers only opened part way. The
> linkage was freed by hand with servo ID 1 torque released, and the sweep
> above re-measured the real travel.

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
| `survey` | Releases torque on one joint at a time, records hand-found stops, restores torque after each joint. Waits for the DDS subscription, samples one servo per request, prints the live pulse every 0.75 s, warns 30 pulses before each EEPROM bound, and aborts when a released joint falls more than 60 pulses between two reads. Requires `--i-have-support` and a TTY. |
| `restore` | Re-engages torque on all five arm servos with a position hold. |

`--high-only` measures only the high-pulse stop and takes the low stop from
the EEPROM limit. It is the **preferred** mode for joints whose low side
lies beyond the register range: it never drives the hand toward 0, where a
negative read kills the vendor node.

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

3. For each joint, type `ready` only after it is supported, then move it
   slowly while watching the live pulse readout. Stop at the first
   mechanical resistance **or at the guard warning** and press Enter while
   holding the joint still — the tool samples the position at Enter, so
   moving back before pressing Enter records the wrong value. Torque is
   restored immediately after the second reading (and in a `finally` block
   on Ctrl-C or on the free-fall guard).
4. Never push a joint into the negative register range. The warning at
   30 pulses before the EEPROM bound is a stop signal, not advice: a
   **negative** position read crashes the vendor node. Use `--high-only`
   when the low side is not needed.
5. Check the printed `repeat spread`, `high_stop_source` and
   `low_stop_source` values. Repeat a joint if the spread is greater than
   5 pulses. `guard_stop` means the recorded value is the guard stop, not a
   mechanical stop; `eeprom_limit` means the bound came from the EEPROM
   table.
6. If anything feels wrong, stop the tool and run `--mode restore`. If the
   vendor node died, keep supporting the arm, move the free joint back into
   the positive range by hand, restart only `ros_robot_controller` (not the
   container), then run `--mode restore`.

The tool publishes `SetBusServoState` only for torque (`[1, 1]` releases,
`[1, 0]` engages on this firmware) and a `ServosPosition` hold at the
current registers for recovery; it never commands a new joint position.

## Adopted limits

The adopted limits were re-verified on 2026-11-19 against the formal survey
in `armpi-debug-studio/configs/joint_mapping.yaml`
(`mapping_version: measured-2026-11-19`,
`verification_status: measured_2026_11_19_operating_envelope`), the URDF
(`armpi_ultra.xacro`), the MoveIt `joint_limits.yaml` and the safety
`limits.yaml`. The rule is the EEPROM limit minus a **20-pulse margin**
(0.0838 rad = 4.8°), or the measured stop minus 20 where that stop is
tighter:

| Joint | Basis | Adopted pulse | Adopted rad |
|---|---|---|---|
| joint1 | EEPROM 50…950 − 20; hand stops 58/957 confirm | 70…930 | −1.8012…1.8012 |
| joint2 | low hand stop 322 + 20; **high hand stop 795 − 20** | 342…**775** | −0.6618…**1.1519** |
| joint3 | vendor one-sided 500…1000; EEPROM 950 − 20 (low-pose hand stop 939) | 500…930 | −1.8012…0.0 |
| joint4 | EEPROM 100…900 − 20; guard stops 130/882 | 120…880 | −1.5917…1.5917 |
| joint5 | EEPROM 100…890 − 20; guard stops 122/869 | 120…870 | −1.5917…1.5499 |

**Why the stops are pose-dependent.** With servo torque released, a joint's
travel is stopped either by a hard stop or by the links touching
(self-collision). joint2's high stop is **795 in the low grasp pose and 873
upright**; joint3's low stop is **939 in the low pose and 923 upright**. A
single static bound cannot be inside every pose's self-collision envelope, so
the adopted values are chosen for the **operating envelope** (the low grasp
family and the view pose) and MoveIt's self-collision checking guards the
other poses. The survey's own numbers are guard-bounded on several sides
(the operator stopped at the 30-pulse line), so those sides are advisory, not
confirmed mechanical stops.

`verified: true` therefore means: the pulse→radion signs/scales were
independently validated by the hand-eye, and every adopted bound is either
inside a hand-measured stop or inside the EEPROM clamp with a guard-bounded
hand check. It does **not** claim that the mechanical stops are hard stops or
configuration-independent.

## Planner margin

The grasp planner keeps a further `--joint-limit-margin` inside every adopted
bound, default **0.03 rad** (1.7°, about 7 servo pulses, ~3.6× the measured
tracking error). The joint-space sampler drops samples inside the margin and
adds the margin boundary, the evaluator rejects an IK solution closer than the
margin as `JOINT_LIMIT_MARGIN`, and IK seeds are clamped into the shrunk range.
This is a planning margin, not a new limit: it does not change the URDF or the
safety service, and `--joint-limit-margin 0.0` reproduces the earlier search.

The trigger was the 2026-11-18 table-alignment plan-only run, whose only plan
sat exactly on `joint2 = −0.6618`. Details and the mock verification are in
[GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) §5.1 and
[`evidence/phase2/onsite_20261118/joint_limit_margin_20261118.md`](../../evidence/phase2/onsite_20261118/joint_limit_margin_20261118.md).

## Limitations

- The bus-servo position is the servo's own register, not an independent
  encoder. It can report outside 0…1000 when a joint is back-driven, but the
  servo only accepts commands inside its EEPROM position limit.
- A hand-found stop is only as repeatable as the operator and the mechanism;
  the tool records the spread but cannot eliminate it.
- Backlash and compliance mean the stop can differ depending on approach
  direction. Measure both directions if that matters.
- `joint3` has a one-sided adopted range (−1.8012…0 rad); do not assume a
  symmetric ±2.09 range. Its positive travel exists mechanically but is not
  commandable under the vendor mapping.
- A negative position read **crashes the vendor node** (uint16
  `AssertionError`); this happened four times on 2026-11-18. Only
  `--high-only`, the live guard and operator discipline prevent it.
- A released joint is not held by the mechanism. joint5 fell 196 pulses in
  0.75 s when released with the arm extended, so its stops cannot be
  hand-measured safely.
- The mechanical stops are self-collision-limited and pose-dependent (joint2
  795/873, joint3 923/939 pulse in two poses); the static limits are verified
  for the operating envelope only.
- Several surveyed sides are guard stops (the operator stopped at the
  30-pulse line), so the true stops there are at or beyond the recorded value.
- joint3's low bound (930 pulse) is 7 pulses beyond the upright guard value
  but 9 pulses inside the low-pose hand stop; the grasp family needs it.
- The gripper (`r_joint`, servo 1) was measured on 2026-11-18 (see above) and
  the five arm joints were formally surveyed on 2026-11-19.
