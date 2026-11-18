# Measured Joint Limits — [P] arm, [V] gripper

> Status: **gripper measured 2026-11-18; arm joints PARTIALLY measured
> 2026-11-18.** The torque path is now understood: the vendor SDK boolean is
> inverted on this servo firmware (`enable_torque=[1, 1]` / 0x0B **releases**,
> `[1, 0]` / 0x0C **engages**). joint2 was measured (322…873 pulse);
> joint1's positive side reached 1164 and a command at 1164 was clamped to
> ~952; joints 3–5 remain unmeasured after a vendor-node crash. The servo
> EEPROM position limits are the effective commandable range. See
> [torque mapping](../../evidence/phase2/onsite_20261118/step6_torque_mapping.md)
> and [partial survey](../../evidence/phase2/onsite_20261118/step6_survey_partial.md).

| Joint | URDF (provisional) | Vendor software | Boot pose 09:43 | Reset pose 09:50 | Measured lower | Measured upper | Status |
|---|---:|---:|---:|---:|---:|---:|---|
| joint1 | -2.09 … 2.09 | -2.094 … 2.094 | **+0.8378** (servo 700) | **0.0** (servo 500) | — | ≥1164 pulse (clamped ~952) | `[P]` |
| joint2 | -2.09 … 2.09 | -2.094 … 2.094 | 0.0 | 0.0 | 322 pulse (−0.746) | 873 pulse (+1.562) | `[M]` |
| joint3 | **-2.09 … 0.0** | -2.094 … 0.0 | 0.0 | 0.0 | — | — | `[U]` |
| joint4 | -2.09 … 2.09 | -2.094 … 2.094 | 0.0 | 0.0 | — | — | `[U]` |
| joint5 | -2.09 … 2.09 | -2.094 … 2.094 | 0.0 | 0.0 | — | — | `[U]` |
| r_joint | **-0.8378 … 0.0** (measured) | -2.932 … 1.257 | -0.8378 (open) | -0.8378 (open) | **-0.8378** (open stop) | **0.0** (closed stop) | `[V]` |

Status legend: `[M]` hand-measured 2026-11-18, `[P]` partial, `[U]`
unmeasured, `[V]` verified. The joint1 upper value is a hand-pushed
mechanical stop beyond the servo range; commanded motion is clamped to the
EEPROM limit (~950 pulse / +1.885 rad).

Servo mapping from the vendor `servo_controller.yaml`: joint1 = ID 6,
joint2 = ID 5, joint3 = ID 4 (min 1000 / max 500 inverted), joint4 = ID 3,
joint5 = ID 2, r_joint = ID 1. At the boot pose, servo ID 6 was 700 and IDs
1–5 were 500; at the reset pose all were 500.
`servo_states.voltage` is 0 and is not a usable measurement.

## What is verified

- The vendor startup init moves **joint1 to +0.8378 rad** (servo ID 6 = 700).
  The other arm joints start at 0 (servo 500).
- The supervised `reset_servo` test moved joint1 back to 0; the other arm
  joints stayed at 0.
- The live `r_joint ≈ -0.8378 rad` is consistent with the physically open
  gripper (servo ID 1 = 500).
- The values above are **commanded** values echoed by `/joint_states` and
  `/controller_manager/servo_states`; they are not encoder feedback. The
  separate bus-servo service does provide a hardware position read (below).

## Actual servo position feedback (2026-11-17)

`/ros_robot_controller/bus_servo/get_state`
(`ros_robot_controller_msgs/srv/GetBusServoState`, `get_position=1`) reads the
position register from each bus servo. Unlike `servo_states`, this is a
hardware read. At the `camera_forward`-derived capture pose it returned
joint1..joint5 = **497, 642, 808, 331, 500** pulses, while the commanded values
were 500, 642, 800, 330, 500. That is joint1 −0.72°, joint3 −1.92°,
joint4 +0.24° from the command. Three reads before/after a 3-frame capture
moved by only 0–1 pulse.

The 2026-11-17 hand-eye calibration uses these measured positions in FK; see
[`docs/phase2/CALIBRATION.md`](../phase2/CALIBRATION.md). They are the servo's
own position register, still not an independent external metrology.

## Gripper measurement (2026-11-18, [V])

Supervised, no-object test through
`/gripper_controller/follow_joint_trajectory`, with the physical position read
from `/ros_robot_controller/bus_servo/get_state` (servo ID 1). Evidence:
[`evidence/phase2/onsite_20261118/`](../../evidence/phase2/onsite_20261118/).

| Observation | Result |
|---|---|
| Sign | **Increasing `r_joint` closes the fingers** (operator-confirmed); `l_joint` mirrors with multiplier −1 |
| Open stop | `r_joint = -0.8378` (command raw 500); servo register settles at **502** |
| Closed stop | `r_joint = 0.0` (command raw 700); servo register settles at **642**, fingers fully closed |
| Usable range | **0.8378 rad** (`-0.8378 … 0.0`) |
| Over-travel probes | `-0.86` / `-0.88` rad moved the register only 508 → 506 pulses → open stop confirmed |
| Command vs register | The register is the **linkage shaft**, not `r_joint`; it moves 502 → 642 while the command moves 500 → 700. `/joint_states` echoes the command, so it cannot confirm contact or stall |
| Empty-close caveat | Commanding `0.0` with no object holds the servo against the closed stop (register 642 while commanded 700). Use it only as the mechanical limit, not as a long-held pose |

The earlier SRDF values (`open: +1.2`, `close: 0`) were wrong on the open side.
They are now `open: -0.80` (about 20 counts inside the open stop) and
`close: 0.0` (mechanical closed limit). The URDF and `joint_limits.yaml` now
use the measured `-0.8378 … 0.0` range.

## Arm-joint survey (2026-11-18) — PARTIAL

The arm was supported, the battery was 7645–7704 mV, and all servos started
torque-enabled. After the torque mapping was corrected, the survey ran:

| Joint | Result | Validity |
|---|---|---|
| joint1 | first run 501…503 (no movement); second run positive side **1164**, guard aborted; hold at 1164 clamped to **952…954** | partial |
| joint2 | **322…873** pulse = −0.746…1.562 rad, span 2.308, spread 1 pulse | measured |
| joint3 | pushed to **−41**; the vendor node then exited with code 1 | not measured |
| joint4, joint5 | not reached | not measured |

The vendor node was restarted in place (not the container) and torque was
restored; the final pose was `joint1=503, joint2=596, joint3=830,
joint4=113, joint5=501`, all torque 1.

The out-of-range joint1 value and the clamp on restore show that the
**servo EEPROM position limits bind for commanded motion**, while the
mechanical stops can be farther out. joint2's low stop (322) is inside its
EEPROM range 160…830 and therefore binds there. Do not force a joint past
the 0…1000 register range: the 2026-11-18 crash happened after such a
sample.

### EEPROM position limits and offsets (read-only, `[V]` firmware bounds)

| Servo | Joint | Position limit (pulse) | Offset |
|---|---|---|---:|
| 1 | r_joint | 150 … 700 | 115 |
| 2 | joint5 | 100 … 890 | −8 |
| 3 | joint4 | 100 … 900 | 13 |
| 4 | joint3 | 70 … 950 | 4 |
| 5 | joint2 | 160 … 830 | −1 |
| 6 | joint1 | 50 … 950 | −7 |

These are the servos' own firmware travel bounds, not hand-measured mechanical
stops; several are wider than the vendor software ranges. They are recorded as
a safety reference only.

## Measurement protocol (requires mechanical support)

1. Torque off / mechanically support the arm.
2. Read each servo's EEPROM offset; record the raw value.
3. Hand-turn each joint to its mechanical stops (where safe); stop at the
   **first mechanical resistance** and never force a joint past the 0…1000
   register range. Record the raw pulse.
4. Convert with the vendor 240°/1000-tick mapping; compare with the table.
5. Record direction and zero offset per joint.
6. Update `joint_limits.yaml` and the URDF only from measured data.
7. Re-run `xacro` + `check_urdf` + a **slow, supervised** single-joint test.

Do not enable full-range arm motion before the arm-joint table is complete.
The gripper range is measured and may be used for supervised tests.
