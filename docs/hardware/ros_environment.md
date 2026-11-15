# ROS Environment — [U]

> Status: **PARTIAL**. The development host is verified; the robot/container is not.

| Item | Value | Evidence | Status |
|---|---|---|---|
| Dev host | R9000P, Ubuntu 24.04, ROS 2 Jazzy | `ls /opt/ros/jazzy` | `[V]` |
| Dev architecture | x86_64 | `uname -m` | `[V]` |
| Robot OS | Raspberry Pi OS, kernel 6.6.20+rpt-rpi | SD rootfs | `[S]` |
| Robot architecture | arm64/aarch64 | SD rootfs | `[S]` |
| Container | `ArmPiUltra` Docker (vendor) | session record | `[U]` |
| Vendor workspace | `/home/ubuntu/ros2_ws` (inside container) | session record | `[U]` |
| Vendor ROS distro | Humble (container) vs Jazzy (dev) | session record | `[U]` |
| DDS / RMW | unknown; CycloneDDS config path reported invalid | session record | `[U]` |
| Domain ID | unknown | — | `[U]` |
| Network | Tailscale `<M70Q_TAILSCALE_IP>` (M70q) only; robot offline | `tailscale status` | `[V]` |

## Blocking gap

The Docker data directory on the SD card is root-only
(`/media/jerichocheng/rootfs/var/lib/docker`). The vendor `/home/ubuntu/ros2_ws`
cannot be inspected until the Board exports a read-only container workspace tar
or grants root read access.
