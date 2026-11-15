# Hardware Baseline — ArmPi Ultra

> Status: **UNVERIFIED**. No real ArmPi hardware was connected during the
> 2026-11-15 audit/merge. Every `[U]` item must be measured on the robot before
> any motion is enabled.

`MASTER_PLAN_V3_PRIVATE.md` §6 requires this folder before Phase 1.

| Document | Purpose |
|---|---|
| [hardware_inventory.md](hardware_inventory.md) | Robot, controller, camera, base, power, stop |
| [ros_environment.md](ros_environment.md) | Host/container, ROS distro, DDS, workspace |
| [driver_interfaces.md](driver_interfaces.md) | Vendor Topic/Service/Action audit |
| [measured_joint_limits.md](measured_joint_limits.md) | Real joint limits and offsets |
| [tf_tree.md](tf_tree.md) | Expected and measured TF tree |
| [camera_mount.md](camera_mount.md) | link4 → camera transform and optical frame |
| [vendor_baseline.md](vendor_baseline.md) | Vendor code/version/hash baseline |

Rules:

1. Vendor code is read-only reference, not copied into this repository.
2. `[U]` cannot be promoted to `Verified` without a command/log/photo.
3. Simulation limits are **provisional**; they are not mechanical limits.
