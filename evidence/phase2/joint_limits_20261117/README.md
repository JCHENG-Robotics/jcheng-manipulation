# Joint limit read-only evidence (2026-11-17)

> Status: **read-only only.** Bus-servo positions and torque states were read on
> the real robot. No torque was disabled, no joint was moved, and no mechanical
> limit was measured. Limits remain provisional.

## Environment

| Item | Value |
|---|---|
| Robot | Pi 5, `raspberrypi`, direct Ethernet `10.42.0.218` |
| Container | `ArmPiUltra`, vendor stack (`bringup` + `vision`) running |
| Battery | 7424 mV |
| DDS | `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml` |
| Tool | `tools/phase2/joint_limit_survey.py --mode read --servo-reads 5` |
| Tool SHA-256 | `336c85332d864f83aa22f5e60b021940ca7980863f684069273a78697784db93` |
| Captured at | `2026-11-17T10:34:45+00:00` |

## Readings

| Joint | Servo ID | Pulse | Joint (rad) | Torque | Provisional range (rad) |
|---|---|---|---|---|---|
| joint1 | 6 | 497 | -0.012566 | 1 | -2.094395 … 2.094395 |
| joint2 | 5 | 642 | 0.594808 | 1 | -2.094395 … 2.094395 |
| joint3 | 4 | 808 | -1.290147 | 1 | -2.094395 … 0.000000 |
| joint4 | 3 | 331 | -0.707906 | 1 | -2.094395 … 2.094395 |
| joint5 | 2 | 500 | 0.000000 | 1 | -2.094395 … 2.094395 |

Raw payload: [`joint_limits_read_20261117.json`](joint_limits_read_20261117.json).

## Interpretation

- All servo torque registers read `1` (enabled).
- All positions are inside the vendor software ranges.
- Positions are consistent with the controller command echo (`joint_states`)
  within the documented command-vs-bus-servo scatter; this checks the servo
  mapping, **not** the mechanical travel.
- The hand-eye calibration independently supports the joint sign/offset
  convention (2.94 mm / 0.39° board scatter over 14 poses using measured servo
  positions). See `docs/phase2/CALIBRATION.md`.

## Not verified

- Mechanical minimum/maximum for any joint.
- Gripper (`r_joint`, servo 1).
- Torque-off behaviour or backdrivability.
- Reboot persistence of the gateway service (deferred by the operator).
