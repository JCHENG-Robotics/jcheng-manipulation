# Measured Joint Limits — [U]

> Status: **NOT MEASURED**. The table records two live command snapshots
> (2026-11-17) and the provisional vendor/URDF values. No joint was moved to a
> mechanical stop for measurement.

| Joint | URDF (provisional) | Vendor software | Boot pose 09:43 | Reset pose 09:50 | Measured lower | Measured upper | Status |
|---|---:|---:|---:|---:|---:|---:|---|
| joint1 | -2.09 … 2.09 | -2.094 … 2.094 | **+0.8378** (servo 700) | **0.0** (servo 500) | — | — | `[U]` |
| joint2 | -2.09 … 2.09 | -2.094 … 2.094 | 0.0 | 0.0 | — | — | `[U]` |
| joint3 | **-2.09 … 0.0** | -2.094 … 0.0 | 0.0 | 0.0 | — | — | `[U]` |
| joint4 | -2.09 … 2.09 | -2.094 … 2.094 | 0.0 | 0.0 | — | — | `[U]` |
| joint5 | -2.09 … 2.09 | -2.094 … 2.094 | 0.0 | 0.0 | — | — | `[U]` |
| r_joint | **-1.57 … 1.257** | -2.932 … 1.257 | -0.8378 (open) | -0.8378 (open) | — | — | `[U]` |

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
- All values above are **commanded** values echoed by `/joint_states` and
  `/controller_manager/servo_states`; they are not encoder feedback.

## Gripper sign warning

The SRDF named states are `open: r_joint = +1.2` and `close: r_joint = 0`, but
the live open pose is `r_joint = -0.8378` and the vendor `r_joint` init is
servo 700 (r_joint 0). The sign convention is therefore **not confirmed**.
Do not use the named gripper states on hardware until a supervised, no-object
open/close test establishes the sign and the safe range. The MoveIt `arm` group
does not move the gripper, so this did not affect the arm motion test.

## Measurement protocol (requires mechanical support)

1. Torque off / mechanically support the arm.
2. Read each servo's EEPROM offset; record the raw value.
3. Hand-turn each joint to its mechanical stops (where safe); record the raw
   pulse.
4. Convert with the vendor 240°/1000-tick mapping; compare with the table.
5. Record direction and zero offset per joint.
6. Update `joint_limits.yaml` and the URDF only from measured data.
7. Re-run `xacro` + `check_urdf` + a **slow, supervised** single-joint test.

Do not enable full-range motion before this file is complete.
