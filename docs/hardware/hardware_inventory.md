# Hardware Inventory — [U]

> Status: **UNVERIFIED**. Fill from the real robot; do not infer from vendor docs.

| Item | Value | Evidence | Status |
|---|---|---|---|
| Arm model | ArmPi Ultra (5-DoF + gripper) | vendor docs | `[S]` |
| Main compute | Raspberry Pi 5 / ROCK 5T (conflicting sources) | vendor docs | `[U]` |
| Controller board | STM32 (vendor firmware) | `ArmPiUltra/STM32` | `[S]` |
| Serial device | `/dev/rrc` expected; not present on dev host | G0 01/03 | `[U]` |
| Bus servos | IDs 1–6 per vendor `servo_controller.yaml` | G0 04 | `[S]` |
| Gripper | `r_joint`, servo ID 1; mimic linkage | vendor URDF | `[S]` |
| Camera | depth camera; model/interface unknown | PRD | `[U]` |
| Base | Mecanum chassis; `CHASSIS_TYPE` unknown | vendor packages | `[U]` |
| E-stop / power cut | physical stop not verified | — | `[U]` |
| Power | battery/voltage/current limits unknown | — | `[U]` |

## To collect

- Photos of the arm, controller, camera mount, base and stop.
- Servo IDs and EEPROM offsets (read-only tool).
- Camera model, serial, firmware, topics.
- Power/battery specs and brown-out behavior.
