# URDF policy

Use `armpi_ultra.xacro` only. It includes `gripper.urdf.xacro`.

The pre-generated `armpi_ultra.urdf` shipped by the vendor was moved to
`docs/archive/armpi_ultra.urdf.stale`: it defines `joint1` and `r_joint` as
fixed and does not match the xacro/mimic model. Do not re-add it as a build
artifact; regenerate from xacro if a URDF file is ever needed.
