# Four-direction grasp working radius — 2026-11-19

> Status: **measured in three directions.** With the robot on the floor and
> the arm at an 18 cm-high far-reach pose, joint1 was yawed to 0°, +88.6° and
> −89.3°. The operator's tape reads a **shoulder-relative grasp radius of
> 205–210 mm** in all three directions and a **grasp height of 180–185 mm**.
> The physical grasp point is the finger-pad centre, about **27 mm inside**
> the URDF nominal `end_effector_link` along the tool axis. joint1 covers
> **±103.2° (206°)**; the remaining directions need a chassis yaw.

## Setup

- Robot on the floor; battery was 7812 mV before the yaw sweep.
- The 13 cm far-reach pose cannot be yawed in place: at that height the
  gripper/finger geometry is only a few mm above the chassis top
  (`base_link` mesh z max 0.0695 m), so the arm was first raised to an
  18 cm pose:
  - target `joint2=368, joint3=612, joint4=120, joint5=500` pulse
  - actual after the raise `joint2≈361, joint3≈620, joint4≈123`
  - nominal TCP `z=+0.092 m`, shoulder radius `0.210 m`
- Joint1 yaw targets were 0°, +90°, −90° (pulse 498/875/125). The measured
  final yaws were +88.6°, 0°, −89.3°.
- The arm was folded to the init pose and joint1 returned to 0° afterwards.

## Operator measurements

| Direction (joint1) | Shoulder → pad | Base centre → pad | Pad height |
|---|---:|---:|---:|
| 0° | **210 mm** | **215 mm** | **180 mm** |
| +88.6° | **205 mm** | **210 mm** | **185 mm** |
| −89.3° | **210 mm** | **215 mm** | **185 mm** |

The spread is 5 mm across directions. The shoulder-relative radius is
therefore direction-independent within the tape resolution.

## Model at the measured poses

`nominal` is the URDF `end_effector_link` (80 mm past link5); `pad` is the
finger-pad centre, 27 mm back along the tool axis. `r_base` is from the
`base_link` origin; `r_shoulder` is from the joint1 axis.

| Direction | nominal r_base | nominal r_shoulder | pad r_base | pad r_shoulder | pad z |
|---|---:|---:|---:|---:|---:|
| 0° | 0.2772 | 0.2102 | 0.2652 | 0.1983 | +0.1133 |
| +88.6° | 0.2222 | 0.2104 | 0.2104 | 0.1980 | +0.1168 |
| −89.3° | 0.2198 | 0.2083 | 0.2096 | 0.1976 | +0.1055 |

- The **shoulder-relative radius is constant** (0.198 m pad, 0.210 m
  nominal); the measured 205–210 mm matches it within 7–12 mm.
- `r_base` varies with direction because the joint1 axis is **67 mm ahead of
  the `base_link` origin** (joint1 origin x = 0.066956 m). The operator's
  "base centre" is near the shoulder, so their 215 mm tracks the shoulder
  radius, not the 0.265 m `base_link` radius.
- The pad point is the actual grasp point: it is ~27 mm inside the nominal
  TCP along the tool and ~25 mm higher. This explains the earlier
  forward-pose readings (195/200/150 mm) and why the nominal 289/222 mm
  reach is not the grasp radius.

## Findings

1. **Farthest normal grasp radius: ~205–210 mm from the shoulder / ~215 mm
   from the operator's base-centre reference**, essentially independent of
   direction; grasp height ~180–185 mm at the 18 cm pose.
2. **joint1 covers ±1.8012 rad = ±103.2° (206.4°)**. The front, left and
   right directions are reachable; the rear ±180° is not. A full 360° sweep
   needs the mecanum chassis yaw (or a robot relocation).
3. **Do not use the nominal `end_effector_link` radius for grasp reach.**
   It is 27 mm beyond the finger-pad centre; using it would make the planner
   aim at a point the fingers cannot hold. The physical pad point should be
   the reach target, or the TCP offset should be reduced accordingly.
4. The 18 cm pose is the safe yaw height with the adopted limits; at 20 cm
   there is no top-grasp solution within the adopted limits
   (`joint4`/`joint5` saturate), and at 13 cm the arm grazes the chassis.
5. Servo tracking errors under the extended load: joint2/3 lagged the
   command by 5–10 pulses throughout (the raise step reported `STALL` at
   7–8 pulses). These are load/compliance errors, not mechanical stops;
   the physical tape is the ground truth at this extension.

## Final state and the stale `/joint_states`

The arm was folded to the init pose at joint1 = 0°, actual registers
`joint1=493, joint2=591, joint3=838, joint4=115, joint5=500`
(nominal TCP `z=+0.109 m`; safe). `joint4=115` is 5 pulses below the adopted
low 120 because the servo could not quite reach 120 at this pose.

**`/joint_states` still reports the last MoveIt command (the far-reach
15 cm pose, `joint5=−1.55`), not the actual folded pose.** A partial
restart or direct `ServosPosition` moves do not refresh the `servo_controller`
state. Before the next MoveIt plan, either:

- restart the full vendor stack (`start_node.service`) so the state
  refreshes at the boot pose, or
- send a `FollowJointTrajectory` goal at the actual registers so the
  `servo_controller` republishes the correct state.

Do not plan from the stale `/joint_states`: it differs from the actual pose
by ~2 rad on joint2/joint3/joint5.

## Files

- Operator readings and model table: this file.
- No new trajectory JSONs: the yaw moves used `pulse_move.py` direct
  `ServosPosition` commands, not MoveIt.
