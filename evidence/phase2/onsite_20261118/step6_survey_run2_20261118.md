# Step 6 — arm joint-limit survey, run 2 (2026-11-18)

Supported, supervised session at the robot. The arm was held by a stand and
an operator throughout. No object in the gripper. This run continues
[step6_survey_partial.md](step6_survey_partial.md) after the torque mapping
was settled in [step6_torque_mapping.md](step6_torque_mapping.md).

## Status

**Partial, stopping on safety grounds.** joint4's high stop was measured;
joint3's range was observed by hand; joint5 was not measurable because the
released wrist falls under gravity. Three more vendor-node crashes happened
during this run, all from a **negative position read** — the root cause is
now identified exactly, not merely suspected. The measured/EEPROM bounds
were adopted downstream with a 20-pulse margin, but the mapping stays
`verified: false` because joint1 (low), joint3 and joint5 are not formally
measured.

## Tool versions

| Stage | SHA-256 of `joint_limit_survey.py` |
|---|---|
| Live monitor + `--high-only` (runs below) | `cad8047e5dd74548888a47fed1ddb7fb1122a3ff08830ec9a998546f17f9b79f` |
| Final (adds the free-fall guard) | `fd0fab858163735a3a64760dbb1698d05e1380e605ded5568287f79ab42c06d2` |

The run started from a state read at **02:45 UTC**: battery 7629 mV, all
torques 1, pose `joint1=500, joint2=598, joint3=830, joint4=112,
joint5=501` (see [joint_limits_recovered_20261118.json](joint_limits_recovered_20261118.json);
the pre-run values were 498/596/830/111/500).

## Results

| Joint | Servo | Attempt | Result | Validity |
|---|---|---:|---|---|
| joint4 | 3 | `--high-only` | **high stop 901 pulse** (+1.680 rad), repeat spread 1 pulse; low recorded from the EEPROM limit (100) | **valid** — [joint_limits_joint4_20261118.json](joint_limits_joint4_20261118.json) |
| joint3 | 4 | both sides | hand travel **23 … 940 pulse** (+1.998 … −1.843 rad) with no hard stop; the negative read then killed the node | observed, no JSON record |
| joint5 | 2 | `--high-only` | 94 → **875** → operator returned to 257 before Enter; the record would be 257, not a stop | **invalid** — [joint_limits_joint5_20261118_invalid.json](joint_limits_joint5_20261118_invalid.json) |
| joint5 | 2 | retry `--high-only` | released at 258 and **fell to 62 in one 0.75 s read**; the next read was negative and killed the node | **no data** |

### joint3 (hand travel, no formal stop)

The first attempt recorded 895 five times (first resistance), then the
operator pushed through to a negative register. The retry reached 940 on
the high side and 23 on the low side. That proves both stops lie outside
the commandable EEPROM range `70..950`, so the EEPROM bounds bind for
commanded motion. No stop value is claimed.

### joint5

The wrist was released twice while the arm was extended (joint4 ≈ 894).
The second time it dropped 196 pulses between two reads, i.e. the released
joint is not held against gravity by the mechanism. A hand stop cannot be
measured safely this way; the EEPROM bounds `100..890` are used instead.

## Vendor-node crashes: exact root cause

All three crashes in this run — and the earlier crash in
[step6_survey_partial.md](step6_survey_partial.md) — have the same cause:
the vendor service builds a `BusServoState` whose `position` field is
`uint16`, so a **negative** position register raises an `AssertionError`
inside the service callback and the node exits with code 1. The journal
from the first crash (10:27 local) captured the traceback:

```
[ros_robot_controller-1]   File ".../ros_robot_controller_node.py", line 230, in get_bus_servo_state
[ros_robot_controller-1]     data.position = state
[ros_robot_controller-1]   File ".../msg/_bus_servo_state.py", line 266, in position
[ros_robot_controller-1]     assert ...
[ros_robot_controller-1] AssertionError: The 'position' field must be a set or sequence
  and each value of type 'int' and each unsigned integer in [0, 65535]
[ros_robot_controller-1] [ERROR] [ros_robot_controller-1]: process has died [pid 1192, exit code 1]
```

Consequences:

- A **positive** out-of-range value is harmless (joint1 at 1164 read back
  fine); only a negative value crashes the node, and it crashes during the
  very read that returns it.
- A client-side guard cannot prevent the crash; it only turns a subsequent
  read into a clean abort. The only prevention is to keep the register
  positive: stop the hand at the guard, or catch a falling joint before the
  next read.

### Tool hardening added during this run

1. `first[name]` / `second[name]` in `survey_one()` — a hardening-era
   refactor bug (`sample_joint` returns two integers). Never reached before
   because every earlier run aborted inside the guard; fixed before this run.
2. **Live pulse readout** every 0.75 s while the operator moves the joint,
   with a warning 30 pulses before each EEPROM bound.
3. **`--high-only`**: measure only the high-pulse stop and take the low stop
   from the EEPROM limit, keeping the joint away from the negative range.
4. **Free-fall guard** (final version, after the joint5 incident): abort
   when a released joint drops more than 60 pulses between two reads. The
   exception path restores torque at the current position, so the joint is
   caught before it reaches 0.

## Recovery procedure used (twice)

1. Keep supporting the arm; the released joint is free and the node is dead.
2. Move the free joint back into the positive register range by hand.
3. Restart **only** `ros_robot_controller` (never the container or the Pi —
   the vendor startup check moves the arm):
   `docker exec -d -u ubuntu ArmPiUltra /bin/zsh -c 'source ~/.zshrc;
   exec .../ros_robot_controller --ros-args'`
4. Read one servo per request (`read_one_servo.py`) so a still-negative
   joint cannot take down the node while the others are checked.

On restart the node re-enables torque on all five arm servos and each servo
holds its last commanded position; a joint left far from that target can
move when torque returns, so it must be supported first.

Final pose after the session: `joint1=498, joint2=596, joint3=834,
joint4=109, joint5=500`, all torque 1, battery 7629–7690 mV.

## Adopted limits (downstream change)

`min_raw`/`max_raw` in `armpi-debug-studio/configs/joint_mapping.yaml`, the
URDF and the MoveIt `joint_limits.yaml` now use the EEPROM limit minus a
20-pulse margin (20 pulses = 0.0838 rad = 4.8°), except joint2's low side,
where the hand-measured stop is inside the EEPROM range:

| Joint | Basis | Adopted pulse | Adopted rad |
|---|---|---|---|
| joint1 | EEPROM 50…950 − 20 | 70…930 | −1.8012…1.8012 |
| joint2 | low stop 322 + 20; high EEPROM 830 − 20 | 342…810 | −0.6618…1.2985 |
| joint3 | vendor one-sided 500…1000; EEPROM 950 − 20 | 500…930 | −1.8012…0.0 |
| joint4 | EEPROM 100…900 − 20 | 120…880 | −1.5917…1.5917 |
| joint5 | EEPROM 100…890 − 20 | 120…870 | −1.5917…1.5499 |

`verified` stays **false** at file and joint level. Unmeasured assumptions:

- joint1's low stop (EEPROM 50 assumed to bind) and joint4's low stop.
- joint3's and joint5's mechanical stops (observed outside/inside the
  commandable range, but not measured as stops).

Hardware motion remains disabled by the `mapping_verified=false` gate.
