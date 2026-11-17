# Hardware Baseline — ArmPi Ultra

> Status: **LIVE BASELINE CAPTURED (2026-11-17)**. The robot was connected over
> direct Ethernet; read-only queries plus one supervised `reset_servo` motion.
> Camera RGB/depth streams and intrinsics are verified. Joint limits, the IMU
> and the camera hand-eye extrinsic remain open. `[U]` items must not be
> treated as measured.

`MASTER_PLAN_V3_PRIVATE.md` §6 requires this folder before Phase 1.

| Document | Purpose |
|---|---|
| [hardware_inventory.md](hardware_inventory.md) | Robot, controller, camera, base, power, stop |
| [ros_environment.md](ros_environment.md) | Host/container, ROS distro, DDS, workspace |
| [driver_interfaces.md](driver_interfaces.md) | Vendor Topic/Service/Action audit |
| [measured_joint_limits.md](measured_joint_limits.md) | Real joint limits and offsets |
| [tf_tree.md](tf_tree.md) | Live and model TF trees |
| [camera_mount.md](camera_mount.md) | `link4 → camera` transform, frames, intrinsics |
| [vendor_baseline.md](vendor_baseline.md) | Vendor code/version/hash baseline |

## Phase 1 baseline summary

| Area | State |
|---|---|
| Host / OS / container | Raspberry Pi 5, Debian 12, kernel 6.6.20, Humble container `ArmPiUltra` |
| ROS graph | one `/controller_manager`, one `/servo_manager`, one arm action server, one MoveIt client; `/joint_states` ~20 Hz |
| Real-robot MoveIt | Humble Docker MoveIt/RViz; supervised `reset_servo` verified |
| Camera | Aurora 930; RGB 8.0 Hz `bgr8`, depth 8.2 Hz `mono16` (mm); intrinsics read |
| TF | `base_link → … → rgb_camera_link` / `depth_camera_link` verified; no `camera_optical_frame` |
| Joint limits | **not measured**; two command snapshots recorded |
| IMU | topic exists but no messages observed |
| Battery | 7.651 V latest |
| Root disk | 95% used |

Rules:

1. Vendor code is read-only reference, not copied into this repository.
2. `[U]` cannot be promoted to `Verified` without a command/log/photo.
3. Simulation limits are **provisional**; they are not mechanical limits.
4. `joint_states` and `servo_states` are cached commands, not measured encoder
   feedback.
