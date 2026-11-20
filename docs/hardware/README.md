# Hardware Baseline — ArmPi Ultra

> Status: **LIVE BASELINE CAPTURED 2026-11-17; updated through 2026-11-20.**
> Since 2026-11-18 20:20 HKT the **direct Ethernet link**
> (`10.42.0.218` ↔ `10.42.0.1`) is the only inbound path: the robot runs a
> default-deny firewall and office WiFi (`10.51.128.226`) is outbound-only.
> See `network_access.md`. The initial baseline was read-only plus one
> supervised `reset_servo`; later sessions added formal joint-limit surveys,
> supervised MoveIt execution, a re-measured gripper, 3/5 first-series real
> pick–lift–place attempts, a pre-registered 24-pair trial (12/24 → 24/24),
> and a chassis-front D435i with depth/colour services.
> Camera RGB/depth streams and intrinsics are
> verified. The camera optical extrinsic is **validated** (14-pose chessboard
> hand-eye, 2.94 mm / 0.39° board scatter). Joint limits are formally surveyed
> on all five joints and adopted from the servo EEPROM with a 20-pulse margin
> (`mapping_verified=true` for the operating envelope); the IMU and encoders
> are absent.
> `[U]` items must not be treated as measured.

`MASTER_PLAN_V3_PRIVATE.md` §6 requires this folder before Phase 1.

| Document | Purpose |
|---|---|
| [hardware_inventory.md](hardware_inventory.md) | Robot, controller, camera, base, power, stop |
| [ros_environment.md](ros_environment.md) | Host/container, ROS distro, DDS, workspace |
| [driver_interfaces.md](driver_interfaces.md) | Vendor Topic/Service/Action audit |
| [measured_joint_limits.md](measured_joint_limits.md) | Real joint limits and offsets |
| [tf_tree.md](tf_tree.md) | Live and model TF trees |
| [camera_mount.md](camera_mount.md) | `link4 → camera` transform, frames, intrinsics |
| [camera_frames.md](camera_frames.md) | Body/optical frame contract and `camera_optical_calibrated` |
| [vendor_baseline.md](vendor_baseline.md) | Vendor code/version/hash baseline |
| [vendor_calibration_baseline.md](vendor_calibration_baseline.md) | Decoded vendor table/hand-eye matrices; Baseline A status |

## Phase 1 baseline summary

| Area | State |
|---|---|
| Host / OS / container | Raspberry Pi 5, Debian 12, kernel 6.6.20, Humble container `ArmPiUltra` |
| ROS graph | one `/controller_manager`, one `/servo_manager`, one arm action server, one MoveIt client; `/joint_states` ~20 Hz |
| Real-robot MoveIt | Humble Docker MoveIt/RViz; supervised `reset_servo` and repeated grasp executions verified; 3/5 first-series attempts and 24/24 in the paired trial |
| Camera | Aurora 930 wrist camera; RGB **~12 Hz cross-host** (`bgr8`, 640×400; in-container subscribers undercount to ~8 Hz), depth 8.2 Hz `mono16` (mm) prior in-container; intrinsics read. Chassis-front **RealSense D435i** on USB 3.0: colour 1280×720, depth 640×480 Z16 ~62 % valid, IMU unavailable |
| Front camera (D435i) | **Calibrated 2026-11-20**: fx = fy 903.5 px / RMS 0.371 px; `base_link → ext_cam_optical` = [0.136, 0.027, 0.038] m, 31.6° pitch; `armpi-rs-depth` + `armpi-ext-cam`; depth starts ~0.2 m; MoveIt collision box |
| Clock sync | **Laptop-pushed systemd user timer every 60 s (2026-11-20)**: restricted forced-command key, Pi held ~2 s ahead; reboot test PASS |
| TF | `base_link → … → rgb_camera_link` / `depth_camera_link` verified; measured `camera_optical_calibrated` published; no live model `camera_optical_frame` |
| Camera optical extrinsic | **Validated** 14-pose chessboard hand-eye: 2.94 mm / 0.39° board scatter; held-out 3.29 mm / 0.43°; body→optical 89.798° |
| Joint limits | **Formally surveyed 2026-11-19**: all five joints hand-measured with torque released (joint1 58…957, joint2 327…795 low pose / 186…873 upright, joint3 100…923, joint4 130…882, joint5 122…869); adopted EEPROM−20 with joint2 high tightened to 775; `mapping_verified=true` for the operating envelope |
| Gripper | Stroke **2.22 rad** (open −2.22, closed 0.0), ~34 counts backlash; measured 2026-11-18 |
| Controller board | Battery, 6 bus servos, 4 open-loop motor duties, buzzer and LED1/2 real; **IMU, encoders, e-stop, onboard gamepad/SBUS absent** (probed 2026-11-19) |
| Battery | **8365 mV** at the 2026-11-20 throughput start; 7812 mV at the 2026-11-19 baseline; 7.651 V at the 2026-11-17 baseline |
| Root disk | 84.7% used, ≈6.6 GiB free (2026-11-17 Session 4) |

Rules:

1. Vendor code is read-only reference, not copied into this repository.
2. `[U]` cannot be promoted to `Verified` without a command/log/photo.
3. Simulation limits are model limits, not mechanical guarantees. The adopted
   commandable limits are EEPROM-bounded and verified for the operating
   envelope (2026-11-19).
4. `joint_states` and `servo_states` are cached commands, not measured encoder
   feedback. The separate
   `/ros_robot_controller/bus_servo/get_state` service does read the servo
   position register and is used by the hand-eye calibration.
