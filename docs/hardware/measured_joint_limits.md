# Measured Joint Limits — [U]

> Status: **NOT MEASURED**. The values below are provisional intersections of
> vendor software mapping and vendor URDF, not mechanical limits.

| Joint | URDF (merged) | Vendor software | Measured lower | Measured upper | Offset | Status |
|---|---:|---:|---:|---:|---:|---|
| joint1 | -2.09 … 2.09 | -2.094 … 2.094 | — | — | — | `[U]` |
| joint2 | -2.09 … 2.09 | -2.094 … 2.094 | — | — | — | `[U]` |
| joint3 | **-2.09 … 0.0** | -2.094 … 0.0 | — | — | — | `[U]` |
| joint4 | -2.09 … 2.09 | -2.094 … 2.094 | — | — | — | `[U]` |
| joint5 | -2.09 … 2.09 | -2.094 … 2.094 | — | — | — | `[U]` |
| r_joint | **-1.57 … 1.257** | -2.932 … 1.257 | — | — | — | `[U]` |

## Measurement protocol (requires Board approval)

1. Torque off / mechanically supported.
2. Read each servo's EEPROM offset; record raw value.
3. Hand-turn each joint to its mechanical stops (where safe); record raw pulse.
4. Convert with the vendor 240°/1000-tick mapping; compare with the table.
5. Record direction and zero offset per joint.
6. Update `joint_limits.yaml` and the URDF only from measured data.
7. Re-run `xacro` + `check_urdf` + a **slow, supervised** single-joint test.

Do not enable full-range motion before this file is complete.
