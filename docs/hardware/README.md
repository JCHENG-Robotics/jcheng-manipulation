# Hardware Baseline — ArmPi Ultra

> Status: **LIVE BASELINE CAPTURED (2026-11-17)**. Since 2026-11-18 20:20 HKT
> the **direct Ethernet link** (`10.42.0.218` ↔ `10.42.0.1`) is the only
> inbound path: the robot runs a default-deny firewall and office WiFi
> (`10.51.128.226`) is outbound-only. See `network_access.md`. Read-only
> queries, one supervised `reset_servo` motion and one plan-only
> `camera_forward` check. Camera RGB/depth streams and intrinsics are
> verified. The camera optical extrinsic is **validated** (14-pose chessboard
> hand-eye, 2.94 mm / 0.39° board scatter). Joint limits and the IMU remain
> open. `[U]` items must not be treated as measured.

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
| Real-robot MoveIt | Humble Docker MoveIt/RViz; supervised `reset_servo` verified; `camera_forward` plan-only verified |
| Camera | Aurora 930; RGB **~12 Hz cross-host** (`bgr8`, 640×400; in-container subscribers undercount to ~8 Hz), depth 8.2 Hz `mono16` (mm) prior in-container; intrinsics read |
| TF | `base_link → … → rgb_camera_link` / `depth_camera_link` verified; measured `camera_optical_calibrated` published; no live model `camera_optical_frame` |
| Camera optical extrinsic | **Validated** 14-pose chessboard hand-eye: 2.94 mm / 0.39° board scatter; held-out 3.29 mm / 0.43°; body→optical 89.798° |
| Joint limits | **not measured**; two command snapshots recorded |
| IMU | topic exists but no messages observed |
| Battery | 7.651 V latest |
| Root disk | 84.7% used, ≈6.6 GiB free (2026-11-17 Session 4) |

Rules:

1. Vendor code is read-only reference, not copied into this repository.
2. `[U]` cannot be promoted to `Verified` without a command/log/photo.
3. Simulation limits are **provisional**; they are not mechanical limits.
4. `joint_states` and `servo_states` are cached commands, not measured encoder
   feedback. The separate
   `/ros_robot_controller/bus_servo/get_state` service does read the servo
   position register and is used by the hand-eye calibration.
