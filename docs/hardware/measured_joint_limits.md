# Measured Joint Limits — [P] arm, [V] gripper

> Status: **gripper measured 2026-11-18; arm joints PARTIALLY measured
> 2026-11-18; forward-facing lowest reach measured 2026-11-19.** The torque
> path is understood: the vendor SDK boolean is
> inverted on this servo firmware (`enable_torque=[1, 1]` / 0x0B **releases**,
> `[1, 0]` / 0x0C **engages**). joint2 was measured (322…873 pulse),
> joint4's high stop reached 901, joint3 was observed over 23…940, and
> joint5's released wrist falls under gravity so it could not be measured.
> joint1's positive side reached 1164 and a command at 1164 was clamped to
> ~952. The servo EEPROM position limits are the effective commandable
> range. **A negative position read crashes the vendor node** (uint16
> assertion), which happened four times. See
> [torque mapping](../../evidence/phase2/onsite_20261118/step6_torque_mapping.md),
> [partial survey](../../evidence/phase2/onsite_20261118/step6_survey_partial.md)
> and [survey run 2](../../evidence/phase2/onsite_20261118/step6_survey_run2_20261118.md).

| Joint | URDF (adopted) | Vendor software | Boot pose 09:43 | Reset pose 09:50 | Measured lower | Measured upper | Status |
|---|---:|---:|---:|---:|---:|---:|---|
| joint1 | **-1.8012 … 1.8012** | -2.094 … 2.094 | **+0.8378** (servo 700) | **0.0** (servo 500) | — (EEPROM 50 assumed) | ≥1164 pulse (clamped ~952) | `[P]` |
| joint2 | **-0.6618 … 1.2985** | -2.094 … 2.094 | 0.0 | 0.0 | 322 pulse (−0.746) | 873 pulse (+1.562) | `[M]` |
| joint3 | **-1.8012 … 0.0** | -2.094 … 0.0 | 0.0 | 0.0 | travel reached 23 pulse, no stop | travel reached 940 pulse, no stop | `[P]` |
| joint4 | **-1.5917 … 1.5917** | -2.094 … 2.094 | 0.0 | 0.0 | — (EEPROM 100 assumed) | 901 pulse (+1.680) | `[P]` |
| joint5 | **-1.5917 … 1.5499** | -2.094 … 2.094 | 0.0 | 0.0 | travel reached 62, then fell negative | travel reached 875 | `[P]` |
| r_joint | **-2.22 … 0.0** (re-measured 2026-11-18 19:59) | -2.932 … 1.257 | -0.8378 (partial open) | -0.8378 (partial open) | **-2.22** (raw 170, register 188) | **0.0** (raw 700, register 640) | `[V]` |

Status legend: `[M]` hand-measured 2026-11-18, `[P]` partial, `[U]`
unmeasured, `[V]` verified. The URDF column shows the adopted commandable
range (EEPROM limit − 20 pulses, or the measured stop + 20 where it is
inside the EEPROM range). joint1's upper value is a hand-pushed mechanical
stop beyond the servo range; commanded motion is clamped to the EEPROM
limit (~950 pulse / +1.885 rad). The file-level `verified` flag for the
mapping stays **false**.

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
- ~~The live `r_joint ≈ -0.8378 rad` is consistent with the physically open
  gripper (servo ID 1 = 500).~~ **Superseded 2026-11-18 19:59:** at
  `r_joint = -0.8378` (raw 500) the fingers are only **partly** open; the
  real open stop is raw ~150. See the gripper re-measurement below.
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

> **Corrected 2026-11-18 19:59.** The first pass below (`open` at raw 500 /
> register 502, range 0.8378 rad) is **wrong**: the servo was stalling on a
> binding linkage, not on the mechanical stop, so `open` opened the fingers
> only part way. The re-measurement follows the table.

| Observation | Result |
|---|---|
| Sign | **Increasing `r_joint` closes the fingers** (operator-confirmed); `l_joint` mirrors with multiplier −1 |
| Closed stop | `r_joint = 0.0` (command raw 700); servo register settles at **640**, fingers fully closed; raw 650 also stalls at 640 |
| **Open stop (re-measured)** | servo register bottoms out at **167**; reached at command raw **~150**. Commanding 160/120/80/40/0 all stall at 167–176 |
| **Adopted open** | `r_joint = -2.22` (command raw **170**, register **188**) — 21 register counts inside the stop, so the servo is not held against it |
| Usable range | **2.22 rad** (`-2.22 … 0.0`), command travel 170…700 = 530 raw counts |
| Command vs register | Linear over the travel: raw 700→640, 600→619, 550→567, 500→520, 450→468, 400→417/383, 350→367, 300→315, 250→266, 200→216, 160→176. Ratio ≈ 0.86 register per raw count, with **~34 counts of backlash** (raw 400 read 417 going open, 383 returning) |
| `/joint_states` caveat | It echoes the **command**, not the linkage: at raw 509 it reported `r_joint = -0.80` ("open") while the fingers were only partly open |
| Binding history | Before re-measurement the linkage bound at register 504–521 and would not travel below 516 on its own; torque was released on ID 1 only (`enable_torque=[1, 1]`), the operator opened the fingers by hand, and the mechanism then travelled freely to 167 |
| Empty-close caveat | Commanding `0.0` with no object holds the servo against the closed stop (register 640 while commanded 700). Use it only as the mechanical limit, not as a long-held pose |

The adopted values are `open: -2.22` (SRDF), `r_joint = -2.22 … 0.0`
(`joint_limits.yaml` and the URDF), and ±2.22 rad on the five mimic joints.
The 0.00418879 rad/raw-count scale is unchanged, so the new bound is
`(170 − 700) × 0.00418879 = −2.220`.

**Pending:** the ~34-count backlash and the 504→516 bind mean the linkage
should be inspected (rack/pinion) before the gripper is used for grasps; a
grasp-width check should be re-run after any mechanical service.

## Arm-joint survey (2026-11-18) — PARTIAL

The arm was supported, the battery was 7629–7704 mV, and all servos started
torque-enabled. Two runs were made after the torque mapping was corrected:

| Joint | Result | Validity |
|---|---|---|
| joint1 | first run 501…503 (no movement); second run positive side **1164**, guard aborted; hold at 1164 clamped to **952…954** | partial |
| joint2 | **322…873** pulse = −0.746…1.562 rad, span 2.308, spread 1 pulse | measured |
| joint3 | hand travel **23…940** pulse, no hard stop; negative reads killed the node twice; no JSON record | observed |
| joint4 | high stop **901** pulse (+1.680 rad), spread 1 pulse; low from the EEPROM limit | high measured, low unmeasured |
| joint5 | hand travel reached 875; released at 258 it **fell to 62 in 0.75 s**; the next read killed the node | not measurable |

The vendor node was restarted in place (not the container) after every
crash and torque was restored; the final pose was `joint1=498,
joint2=596, joint3=834, joint4=109, joint5=500`, all torque 1.

The out-of-range joint1 value and the clamp on restore show that the
**servo EEPROM position limits bind for commanded motion**, while the
mechanical stops can be farther out. joint2's low stop (322) is inside its
EEPROM range 160…830 and therefore binds there. **A negative position
register crashes the vendor node**: `get_state` assigns it to a `uint16`
field, raising an `AssertionError` in the callback (exit code 1). This is
the exact cause of all four crashes; positive out-of-range values are
harmless.

### Adopted commandable limits (2026-11-18)

The URDF and `joint_limits.yaml` now use EEPROM limit − 20 pulses
(0.0838 rad = 4.8°), with the measured joint2 low stop instead of the
EEPROM bound:

| Joint | Adopted pulse | Adopted rad |
|---|---|---|
| joint1 | 70…930 | −1.8012…1.8012 |
| joint2 | 342…810 | −0.6618…1.2985 |
| joint3 | 500…930 | −1.8012…0.0 |
| joint4 | 120…880 | −1.5917…1.5917 |
| joint5 | 120…870 | −1.5917…1.5499 |

These are adopted for planning, **not verified**: joint1's and joint4's
low stops are unmeasured, and joint3/joint5 have no formal stop values.
`mapping_verified` stays false and hardware motion stays disabled.

### Forward-facing lowest reach (2026-11-19)

The robot was placed on the floor and the arm was driven forward
(`joint1 = 0`) to the adopted limit with MoveIt, then in 5-pulse steps to
the mechanical stops with torque on (`tools/phase2/pulse_move.py`).
Evidence:
[`evidence/phase2/onsite_20261119/lowest_reach/`](../../evidence/phase2/onsite_20261119/lowest_reach/).

| Quantity | Adopted limits | Mechanical maximum |
|---|---:|---:|
| joint2 / joint3 pulse | 342 / 929 | **322 / 939** |
| joint4 pulse | 339 | 366 |
| TCP (FK) | (0.196, 0.000, −0.0444) | (0.191, −0.002, −0.0565) |
| Model clearance to fitted plane | 35.0 mm | 23.0 mm |
| Ruler: lowest point / TCP | — (estimated 22 / 27 mm) | **10 / 15 mm** |

- joint2 reached **322** (its hand-measured stop) and held without a stall.
- joint3 stops at **939**; the EEPROM limit **950 is not mechanically
  reachable**. The adopted 930 is 9 pulses inside the stop.
- The fitted plane is ~**8 mm below the physical floor** at the measurement
  point, and the fingertip is ~**5 mm below the nominal TCP**. The safety
  `clearance_m: 0.01` is a TCP-vs-fitted-plane guard, not a 10 mm physical
  fingertip guarantee.
- The adopted limits were **not changed**; `mapping_verified` stays false
  (joint1/joint4/joint5 low stops remain unmeasured).

### Four-direction grasp working radius (2026-11-19)

With the robot on the floor and the arm at an 18 cm-high far-reach pose,
joint1 was yawed to 0°, +88.6° and −89.3°. Evidence:
[`evidence/phase2/onsite_20261119/reach/`](../../evidence/phase2/onsite_20261119/reach/).

| Direction | Shoulder → pad | Base centre → pad | Pad height |
|---|---:|---:|---:|
| 0° | 210 mm | 215 mm | 180 mm |
| +88.6° | 205 mm | 210 mm | 185 mm |
| −89.3° | 210 mm | 215 mm | 185 mm |

- The shoulder-relative grasp radius is **direction-independent** (model pad
  0.198 m; measured 205–210 mm).
- The physical grasp point is the **finger-pad centre**, about **27 mm
  inside** the URDF `end_effector_link` along the tool. The nominal TCP
  radius (0.210–0.222 m shoulder, 0.220–0.289 m `base_link`) overstates it.
- The `base_link` radius varies with direction because the joint1 axis is
  **67 mm ahead** of the `base_link` origin.
- joint1 covers **±1.8012 rad (±103.2°, 206.4°)**; a full 360° needs the
  mecanum chassis yaw.
- The 13 cm far-reach pose cannot be yawed in place (chassis top at
  `z=0.0695 m`); the 18 cm pose is the safe yaw height. There is no
  top-grasp solution at 20 cm within the adopted limits.

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

1. Torque off / mechanically support the arm; a released joint is not held
   by the mechanism (joint5 fell 196 pulses in 0.75 s).
2. Read each servo's EEPROM offset; record the raw value.
3. Hand-turn each joint to its mechanical stops (where safe); stop at the
   **first mechanical resistance**. **Never push toward a negative
   register** — a negative read crashes the vendor node. Use
   `joint_limit_survey.py --high-only` when the low side is not needed,
   and watch the live pulse readout.
4. Convert with the vendor 240°/1000-tick mapping; compare with the table.
5. Record direction and zero offset per joint.
6. Update `joint_limits.yaml` and the URDF only from measured data.
7. Re-run `xacro` + `check_urdf` + a **slow, supervised** single-joint test.

Do not enable full-range arm motion before the arm-joint table is complete.
The gripper range is measured and may be used for supervised tests.
