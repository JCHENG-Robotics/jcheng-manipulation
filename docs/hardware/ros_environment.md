# ROS Environment — ArmPi Ultra

> Status: **VERIFIED on the live robot (2026-11-17)**, except where marked
> `[U]`. The robot runs Humble; the laptop runs Jazzy for model/mock work and a
> Humble Docker container for real MoveIt/RViz.

| Item | Value | Evidence | Status |
|---|---|---|---|
| Dev host | R9000P, Ubuntu 24.04, ROS 2 Jazzy | `ls /opt/ros/jazzy` | `[V]` |
| Dev architecture | x86_64 | `uname -m` | `[V]` |
| Laptop real-robot stack | Docker image `armpi-humble-moveit`, container `armpi-humble` | `docker images/ps` | `[V]` |
| Robot OS | Debian 12 (bookworm), kernel 6.6.20+rpt-rpi-2712 | `/etc/os-release`, `uname` | `[V]` |
| Robot architecture | aarch64 | `uname -m` | `[V]` |
| Container | `ArmPiUltra` (vendor), Humble | `docker ps` | `[V]` |
| Vendor workspace | `/home/ubuntu/ros2_ws` inside the container | `docker exec` | `[V]` |
| Robot ROS distro | Humble | `ROS_DISTRO=humble` | `[V]` |
| Laptop ROS distro | Humble inside Docker; Jazzy on the host only | `ROS_DISTRO` | `[V]` |
| RMW | `rmw_cyclonedds_cpp` on both sides | env | `[V]` |
| Domain ID | 0 | `ROS_DOMAIN_ID` | `[V]` |
| CycloneDDS (robot) | `file:///home/ubuntu/cyclonedds.xml`, **`eth0` only** (since 2026-11-17 evening; backup `.bak_wlan0_20261117`) | container config | `[V]` |
| CycloneDDS (laptop) | `cyclonedds-laptop.xml`, **`enx00e04ca373c7` only** (direct Ethernet); `wlp4s0` is the WiFi fallback | config | `[V]` |
| Robot network | `eth0 10.42.0.218/24` (SSH + DDS, current); `wlan0 10.51.128.226/23` (office WiFi fallback) | `ip addr` | `[V]` |
| CLI daemon | `ROS2CLI_NO_DAEMON=1` in the Humble containers | env | `[V]` |
| Core service | `start_node.service` active; `ArmPiUltra` container up | `systemctl`, `docker ps` | `[V]` |
| Laptop MoveIt launch | `ros2 launch armpi_ultra_moveit_config real_moveit.launch.py` | `tools/humble_moveit/` | `[V]` |

## Live ROS graph (2026-11-17)

Robot-side nodes:

```text
/controller_manager
/servo_manager
/joint_state_publisher
/kinematics
/ros_robot_controller
/robot_state_publisher
```

Laptop-side MoveIt/RViz nodes visible in the same graph:

```text
/move_group
/moveit_simple_controller_manager
/rviz2
```

Camera nodes when `vision.launch.py` is running:

```text
/aurora/aurora
/camera_mount_tf
/depth_camera_to_rgb_camera_link
/rgb_camera_to_color_frame
/rgb_camera_to_depth_optical_frame
```

- Exactly **one** `/controller_manager`, one `/servo_manager`, and one
  `/arm_controller/follow_joint_trajectory` server (one MoveIt client). The
  duplicate-server bug is fixed in the private vendor workspace by removing
  `name='controller_manager'` from `servo_controller.launch.py`.
- `/joint_states` is published at about **20 Hz**.
- The camera publishes RGB at **8.0 Hz** (`bgr8`, 640×400) and depth at
  **8.2 Hz** (`mono16`, 640×400, millimetres).
- Use `real_moveit.launch.py` (move_group + RViz only). Do not run
  `demo.launch.py` or any `mock_components` controller on the same ROS domain
  as the real robot.

## Camera launch (persistent)

```bash
sudo docker exec -d -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; \
   exec ros2 launch bringup vision.launch.py > /tmp/vision.log 2>&1'
```

The `exec` form keeps the launch owned by the container. A
`nohup ros2 launch ... &` inside `docker exec` was reaped when the exec session
closed; its `aurora930_node` became a zombie. Stop it with
`docker exec ... pkill -f vision.launch.py` if needed.

## Known environment issues

| Issue | Detail | Status |
|---|---|---|
| Transient camera open failure | `aurora930_node` failed with `0x21003` before a restart; after the restart it opened and streamed | `[V]` observed, root cause `[U]` |
| IMU topic quiet | `/ros_robot_controller/imu_raw` has a publisher but produced no message in 5–12 s | `[U]` |
| Root filesystem 87% | ≈6.8 GiB free (2026-11-17 evening); boot/fsck and bag-recording risk | `[V]` |
| Wi-Fi module crash | A stale command loaded `rtw88_8821cu` for a Realtek RTL8188GU (`0bda:1a2b`) adapter in CDROM mode; the kernel oops wedged USB, `lsusb` hung, and a soft reboot stuck in shutdown. A restart recovered it. Do not reload that module on this robot. | `[V]` |
| Battery 7.651 V | Charge before long motion trials | `[V]` |
| Camera launch can die | The first persistent launch later exited with SIGKILL; the static TF publishers logged DDS multicast errors. Restart with the persistent `docker exec -d ... exec` form and check `/aurora/aurora`. | `[V]` |
| DDS interface change | The robot's `cyclonedds.xml` was reverted from `wlan0` to `eth0` after the WiFi fallback (`wlan0: does not match an available interface`); backup `/home/ubuntu/cyclonedds.xml.bak_wlan0_20261117`. The laptop config now binds `enx00e04ca373c7`. | `[V]` |

## Reproduce the link

```bash
# direct Ethernet (SSH + DDS); this is the current path
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218

# office-WiFi fallback (switch the robot cyclonedds.xml back to wlan0 first)
ssh -i ~/.ssh/id_ed25519_armpi pi@10.51.128.226

sudo docker exec -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; ROS2CLI_NO_DAEMON=1 ros2 node list'
```
