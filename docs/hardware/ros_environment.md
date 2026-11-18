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
| CycloneDDS (robot) | `file:///home/ubuntu/cyclonedds.xml`, **`eth0` only** (wired, since 2026-11-18 evening; WiFi backup `cyclonedds.xml.wifi.bak`) | container config | `[V]` |
| CycloneDDS (laptop) | `~/jcheng/humble_ws/cyclonedds-wired.xml`, **`enx00e04ca373c7` only** (wired 10.42.0.1, since 2026-11-18 evening; the WiFi version is `*.wifi.bak`); `cyclonedds-laptop.xml` is the older WiFi copy | config | `[V]` |
| Robot network | `eth0 10.42.0.218/24` (**SSH + DDS, current**, 0.2 ms); `wlan0 10.51.128.226/23` (office WiFi: up, gateway HTTP bind, 7 ms) | `ip addr` | `[V]` |
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
- The camera publishes RGB at about **12 Hz** cross-host (`bgr8`, 640×400) and
  depth at 8.2 Hz in-container (`mono16`, 640×400, millimetres). An
  in-container RGB subscription undercounts to ~8 Hz; use a cross-host
  measurement for FPS (2026-11-17 Session 4).
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
| Root filesystem 84.7% | ≈6.6 GiB free (2026-11-17 Session 4); boot/fsck and bag-recording risk | `[V]` |
| Wi-Fi module crash | A stale command loaded `rtw88_8821cu` for a Realtek RTL8188GU (`0bda:1a2b`) adapter in CDROM mode; the kernel oops wedged USB, `lsusb` hung, and a soft reboot stuck in shutdown. A restart recovered it. Do not reload that module on this robot. | `[V]` |
| Battery 7.651 V | Charge before long motion trials | `[V]` |
| Camera launch can die | The first persistent launch later exited with SIGKILL; the static TF publishers logged DDS multicast errors. Restart with the persistent `docker exec -d ... exec` form and check `/aurora/aurora`. | `[V]` |
| DDS interface change | **Wired is current since 2026-11-18 evening.** The office WiFi path (`wlp4s0 10.51.128.246 ↔ wlan0 10.51.128.226`) suffered a ~10-minute **full data-plane outage** (100 % packet loss, ping and every bulk topic dead) while both nodes stayed up, their addresses were unchanged and DDS *discovery* survived — so the laptop received no `/joint_states`/battery/camera data and `move_group` reported `Failed to receive current joint state`. Both ends were switched to the wired link: robot `cyclonedds.xml` → `eth0` (WiFi backup `.wifi.bak`), laptop → `enx00e04ca373c7`; data returned immediately. The earlier `wlan0: does not match an available interface` failure happens when the XML names an interface that is down — bring the interface up **before** restarting the core. Discovery-only with no data is a **link** symptom, not a ROS bug: check `ping` and the interface before touching the graph. | `[V]` |
| Vendor stack restart re-initialises the servos | Restarting `start_node.service` does **not** hold the previous posture: the arm moved on its own from a folded grasp pose to the vendor init pose **700/500/500/500/500** (`joint1 = 0.8378 rad`, others 0) while the servo torque stayed on. Plan the restart only at a safe pose. The stop script only `kill -9`s the ROS processes. | `[V]` |
| DDS graph anomaly | `get_node_names()` in the robot container fails with `Failed to get node names: empty node name returned by the RMW layer`; topic discovery/subscription still works. Camera/static-TF logs repeat `serdata.cpp` errors (`string data is not null-terminated` / `invalid data size`). Root cause unresolved (2026-11-17 Session 4). | `[V]` observed, root cause `[U]` |
| Camera-rate undercount | An in-container subscriber measures ~8.3 Hz for RGB, while a cross-host subscriber on the laptop measures ~11.8–12.0 Hz. Local DDS delivery of large images drops/queues; do not use in-container FPS as the sensor rate. | `[V]` |
| Camera stop leaves a defunct node | `pkill -f vision.launch.py` exits the launch but can leave `aurora930_node` as a defunct process entry; kill it by exact process name or restart the container. The publisher count returns to 0. | `[V]` |

## Reproduce the link

```bash
# wired (SSH + DDS); this is the current path (0.2 ms)
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218

# office WiFi (gateway HTTP bind; was the DDS path until the 2026-11-18 evening
# data-plane outage; to use it for DDS, restore both *.wifi.bak XMLs first)
ssh -i ~/.ssh/id_ed25519_armpi pi@10.51.128.226

sudo docker exec -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; ROS2CLI_NO_DAEMON=1 ros2 node list'
```
