# Vendor Baseline — read-only reference

> The vendor stack is **not** copied into this repository. Paths below are on
> R9000P and are reference-only.

| Item | Path / value | Status |
|---|---|---|
| Vendor root | `/home/jerichocheng/ArmPiUltra` (16G) | `[V]` |
| Vendor source zip | `ROS2/Armpi_ultra-main.zip` (921,882,700 B) | `[V]` |
| Extracted source | `ROS2/Armpi_ultra-main/armpi_ultra` | `[V]` |
| Packages | `3 功能包文件/simulations/` | `[V]` |
| SDK | `ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py` SHA `369b11dd…` | `[V]` |
| Servo mapping | `ros2_ws/src/driver/servo_controller/config/servo_controller.yaml` | `[V]` |
| Vendor URDF | `simulations/armpi_ultra_description/urdf/armpi_ultra.xacro` | `[V]` |
| G0 audit | `docs/armpi/g0/` in this repo | `[V]` |

## Separation rule

- Vendor code: read-only, not modified, not copied.
- Personal code: `ros2_ws/src/armpi_*`, `armpi-debug-studio/`, `evidence/`, `docs/`.
- If a vendor fix is required, record the diff in `docs/hardware/vendor_baseline.md`
  and get Board approval before touching the original.

## Version caution

Two vendor xacro copies exist:

- `3 功能包文件/...` (2025-07-02, SHA `158224…`) — used by the imported sim package.
- `ROS2/Armpi_ultra-main/...` (2026-04-02, SHA `7ce1d0…`) — newer; only the
  `base_link` collision geometry differs.

Do not silently switch; record which version a model/test uses.
