# armpi_mobile_bringup — draft AMR bringup for ArmPi Ultra

> **STATUS: DRAFT / NOT HARDWARE-VALIDATED.** This package is the offline
> skeleton for route A (2D LiDAR + laser odometry), now based on the
> **LSLIDAR M10**. The M10 is acquired but not yet connected or measured; the
> full Nav2/RTAB-Map dependency set is not installed in the robot container.
> Nothing here has been run on hardware.

The goal is a Nav2 stack for the existing ArmPi Ultra mecanum base:

```text
LSLIDAR M10 (serial, 460800 baud) -> /scan
rtabmap_odom/icp_odometry -> /odom + TF odom -> base_link
slam_toolbox (online async) -> TF map -> odom + /map
Nav2 (MPPI, motion_model: Omni) -> /cmd_vel
vendor controller/mecanum -> /ros_robot_controller/set_motor_speeds
```

The arm stack (MoveIt, calibration, perception) is untouched. `base_link`
remains the arm-mount root; no `base_footprint` is introduced.

## What is measured vs provisional

| Value | Value used | Source / status |
|---|---|---|
| Wheelbase | 0.195 m | vendor `controller/mecanum.py` default — not independently tape-measured |
| Track width | 0.22 m | vendor `controller/mecanum.py` default — not independently tape-measured |
| Wheel diameter | 0.0965 m | vendor `controller/mecanum.py` default — not independently tape-measured |
| Base speed scale | duty ≈ 182 × v(m/s) | 2026-11-18 ruler test — measured |
| Chassis L × W × H | 0.26 × 0.22 × 0.10 m | **PROVISIONAL — measure** |
| Chassis center vs `base_link` | z = −0.05 m | **PROVISIONAL — measure** |
| Arm-mount height above floor | 0.12 m | **PROVISIONAL — measure** |
| LiDAR pose vs `base_link` | (0.12, 0, 0.03) m, yaw 0 | **PROVISIONAL — measure** |
| Nav2 footprint | [−0.10, −0.13] … [0.16, 0.13] | **PROVISIONAL — measure chassis corners** |
| M10 range / rate | 12 m cap in config; 10 Hz | vendor datasheet — verify with `/scan` |

The wheel values and the base speed scale are the only chassis numbers backed
by live evidence. Everything else must be measured before planning or
navigating; the M10 datasheet is not a site measurement.

## LSLIDAR M10 baseline

| Item | Value |
|---|---|
| Model | LSLIDAR M10 (single-line, 360°) |
| Range | 10 m @ 10% reflectivity; up to 25 m @ 70%; 0.05 m minimum |
| Scan rate | 10 Hz, 10,000 points/s, 0.36° angular spacing |
| Accuracy | ±3 cm (vendor datasheet) |
| Interface | serial UART at **460800 baud** (fixed for M10 in the driver) |
| Power | 5 V DC, ~400 mA |
| Size / weight | Ø80 × 39 mm, ~200 g |
| Laser | 905 nm, Class I |

The driver we target is **Lslidar/Lslidar_ROS2_driver**, branch
**`LS-S1_V1.0`** (driver v5.2.0, commit
`e23139673c34c51d07053837910020aaa81736fa`, **Apache-2.0**). Its README
explicitly lists ROS 2 Humble support and M10/M10GPS/M10P/N10/N10Plus/N301.
Do **not** use the older `M10/M10-PHY_V1.0` branch: it is GPLv3 and only
tested up to Foxy. The driver is a plain `rclcpp::Node`
(`lslidar_driver_node`), not a lifecycle node, and it publishes `/scan`
immediately when the serial port opens.

## Files

| File | Purpose |
|---|---|
| `config/nav2_params.yaml` | Nav2 Humble params: MPPI Omni, costmaps, planner, behaviors, velocity smoother |
| `config/slam_toolbox.yaml` | online-async mapping, `base_frame: base_link` |
| `config/icp_odometry.yaml` | RTAB-Map ICP laser odometry (`/scan` → `/odom`) |
| `config/lslidar_m10.yaml` | M10 driver params: serial, `/scan`, `laser_link`, 10 Hz |
| `launch/bench_nav.launch.py` | Nav2 + provisional open-loop odom, **no LiDAR, no base control** |
| `launch/amr_bringup.launch.py` | full stack: M10 + ICP odom + SLAM + Nav2, base optional |
| `armpi_mobile_bringup/open_loop_odom.py` | PROVISIONAL `/cmd_vel` integration, bench only |
| `urdf/armpi_mobile.urdf.xacro` | chassis + 4 wheels + `laser_link` added below the existing arm model |

## Hardware to add / connect

Already acquired: **LSLIDAR M10**. Still needed:

- A **USB-to-serial adapter that exposes 5 V/400 mA** (or a separate 5 V
  supply). If the M10 came with a USB adapter, verify it is a data adapter,
  not a charge-only cable. The M10 UART is 460800 baud.
- A rigid mount placing the scan plane above the chassis top, with a clear
  360° view (not blocked by the arm or the front D435i). Record x/y/z/yaw
  relative to `base_link`.
- A stable serial name. The common M10 USB adapter is CP210x
  (`10c4:ea60`):

  ```udev
  # /etc/udev/rules.d/99-lslidar-m10.rules
  SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lslidar_m10"
  ```

  If the adapter is CH340 (`1a86:7523`) or another chip, do **not** copy the
  rule: the robot already has a CH340 for `/dev/rrc` on USB port path
  `1-2:1.0`. Read the adapter's serial with
  `udevadm info -q property -n /dev/ttyUSB1` and pin the rule by serial or
  port path. Verify with `lsusb` after plugging it in.
- A physical power switch / e-stop in reach. The chassis has **no brake and
  no electronic e-stop**; the only protection today is the 0.5 s `cmd_vel`
  watchdog and the duty-60 (~0.33 m/s) cap in the hardened mecanum node.

## Software prerequisites (robot container, ROS 2 Humble)

### Nav2 / RTAB-Map

Still missing (the container has only a partial Nav2 set: `nav2_common`,
`nav2_core`, `nav2_costmap_2d`, `nav2_map_server`, `nav2_msgs`, `nav2_util`,
`nav2_voxel_grid`):

```bash
apt-get install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-rtabmap-odom
```

Already present: `slam_toolbox` (sync/async/localization) and
`robot_localization`.

### LSLIDAR M10 driver (source build, no apt package)

There is **no `ros-humble-lslidar-*` package** in the ROS apt repository.
Build the Apache-2.0 LS-S1 branch in a separate workspace, not the vendor
`/home/ubuntu/ros2_ws`:

```bash
# On a machine with internet (the laptop), get the pinned source:
git clone https://github.com/Lslidar/Lslidar_ROS2_driver.git
cd Lslidar_ROS2_driver
git checkout LS-S1_V1.0
git rev-parse HEAD   # must be e23139673c34c51d07053837910020aaa81736fa

# Copy the tree to the robot container (example), then inside the container:
mkdir -p /home/ubuntu/lslidar_ws/src
# copy Lslidar_ROS2_driver/* into /home/ubuntu/lslidar_ws/src/
cd /home/ubuntu/lslidar_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build dependencies in the robot container:

| Dependency | State |
|---|---|
| `libpcl-dev`, `libboost-dev`, `libyaml-cpp-dev`, `libeigen3-dev` | installed |
| `ros-humble-pcl-conversions`, `ros-humble-ament-index-cpp` | installed |
| `libpcap0.8` (runtime) | installed (1.10.1-4build1) |
| **`libpcap-dev` + `libpcap0.8-dev` (headers/link)** | **missing — mirror has the exact `1.10.1-4build1` arm64 .debs** |

The robot has `libpcap0.8` **1.10.1-4build1**, so use the matching
`libpcap-dev_1.10.1-4build1_arm64.deb` and
`libpcap0.8-dev_1.10.1-4build1_arm64.deb` from
`https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/pool/main/libp/libpcap/`.
`libc6-dev` and `libdbus-1-dev` are already installed. The driver's CMake
links `-lpcap`, so `libpcap-dev` is required even if you never replay a pcap
file. The robot has no internet right now, so download the two .debs on the
laptop and install them before `colcon build`.

### Current install blocker

The robot's WiFi is up but **has no internet** (Tsinghua mirror and Baidu both
time out; rechecked 2026-11-24). The laptop can reach the mirror, so the
options are:

1. restore the robot's outbound internet and `apt-get update && apt-get install ...`; or
2. stage an offline bundle: on the robot run
   `apt-get install --print-uris -y <packages> | grep "^'"` to list the .debs
   (this uses the local apt lists), download them on the laptop, copy them to
   the robot, and install them locally. Note the robot's apt lists are from
   2026-03-26 and the mirror now hosts newer build stamps (the old URLs
   return 404); either update the lists first or resolve the current package
   set from the mirror index.

Decide whether to install inside the running container or into a derived
image. The vendor container is stateful; apt changes disappear if the
container is recreated.

## Build this package

```bash
cd ~/jcheng/jcheng-manipulation/ros2_ws
colcon build --symlink-install --packages-select armpi_mobile_bringup
source install/setup.bash
```

The package is not in the repo's `ros-ci` package list; add it there only
after it has been validated.

## Run

Bench only (wheels off the ground; no LiDAR, no motor controller):

```bash
ros2 launch armpi_mobile_bringup bench_nav.launch.py
```

M10 driver + SLAM + Nav2, base still disabled:

```bash
source /home/ubuntu/lslidar_ws/install/setup.bash   # driver workspace
ros2 launch armpi_mobile_bringup amr_bringup.launch.py \
  start_base:=false \
  laser_port:=/dev/lslidar_m10 \
  laser_x:=0.12 laser_y:=0.0 laser_z:=0.03 laser_yaw:=0.0
```

Only after the staged checks below, in a clear area with the power switch in
hand:

```bash
ros2 launch armpi_mobile_bringup amr_bringup.launch.py start_base:=true
```

`start_base:=true` starts the vendor `controller/mecanum` node, which
subscribes `/cmd_vel` and publishes four open-loop motor duties. The vendor
`ros_robot_controller` node must already be running.

The M10 motor is treated as running at startup. To stop it (and stop
publishing data) publish `Int8 0` on `/motor_control`; `Int8 1` restarts it:

```bash
ros2 topic pub -1 /motor_control std_msgs/msg/Int8 "{data: 0}"
ros2 topic pub -1 /motor_control std_msgs/msg/Int8 "{data: 1}"
```

(The topic is relative to the driver node namespace; this launch runs the
driver at the root namespace. The vendor `lslidar_x10_launch.py` uses `/x10`
and also starts RViz, so it is not used here.)

Useful switches: `start_lidar`, `start_slam`, `start_nav2`, `start_base`,
`odom_source:=icp|open_loop`.

## TF graph

```text
map -> odom        slam_toolbox
odom -> base_link  rtabmap_odom/icp_odometry (or open_loop_odom on the bench)
base_link -> laser_link   static_transform_publisher / URDF
base_link -> link1 ...    vendor robot_state_publisher (arm + cameras)
```

Nav2 `robot_base_frame` is `base_link`. There is no `base_footprint`; the
footprint polygon in `nav2_params.yaml` is expressed in the `base_link`
frame and must be measured.

## Staged verification

1. **Build + static checks**
   - `colcon build --packages-select armpi_mobile_bringup`
   - `xacro urdf/armpi_mobile.urdf.xacro > /tmp/armpi_mobile.urdf` and check it
     loads in RViz.
2. **M10 driver only** (`start_slam:=false start_nav2:=false start_base:=false`)
   - `ros2 topic hz /scan` ≈ 10 Hz.
   - `ros2 topic echo /scan --once`: `header.frame_id == laser_link`,
     `angle_increment` ≈ 0.00628 rad (0.36°), `range_min` 0.05,
     `range_max` 12.0.
   - `ros2 run tf2_ros tf2_echo base_link laser_link` matches the tape
     measure.
   - Rotate the robot by hand: the scan should rotate consistently.
   - Stop/start the motor with `/motor_control` and confirm `/scan` stops and
     resumes.
3. **Laser odometry only** (`start_base:=false`)
   - `ros2 topic echo /odom --once` and `tf2_echo odom base_link`.
   - Push/rotate the robot by hand with the wheels off the ground; check the
     sign conventions (x forward, y left, yaw CCW positive).
4. **SLAM** (`start_base:=false`)
   - RViz: fixed frame `map`, `/map` and `/scan` visible; `map -> odom` exists.
5. **Nav2 output, no motion** (`start_base:=false`)
   - Send a goal; confirm `/cmd_vel` is non-zero and the velocity smoother
     output is bounded (≤ 0.25 m/s, ≤ 0.20 m/s lateral, ≤ 0.60 rad/s).
6. **First motion** (supervised, clear floor)
   - `start_base:=true`, start at 0.1 m/s, one operator on the power switch,
     arm parked, no arm extension.
   - Stop immediately on any unexpected motion; the base is open-loop.

## Known limitations / safety

- **No wheel encoders, no IMU.** The M10 ICP odometry is the only odometry
  source. Mecanum slip and scan-matching drift are expected; this is not a
  production localization solution.
- **M10 range is reflectivity-dependent** (10 m @ 10%, 25 m @ 70%). The
  config caps at 12 m; do not treat far returns as reliable in a dark, glossy
  or reflective room.
- **No hardware e-stop or brake.** Add a physical power switch before driving.
- **No collision monitor.** Nav2 costmaps + the mecanum watchdog are the only
  layers; add `nav2_collision_monitor` later.
- **The front D435i is aimed down at the table** (31.6° pitch, depth starts
  ~0.2 m); it is not a navigation sensor. Keep the M10 as the navigation
  source.
- **Untethered operation is not enabled.** The firewall accepts inbound only
  on `eth0`; `wlan0` inbound is dropped. DDS currently needs the cable. A
  narrow wlan0 rule or a dedicated router is required before calling this a
  mobile robot.
- **Disk is 85% used** (about 7.7 GiB free). Free space before installing
  Nav2/RTAB-Map and before recording bags.
- **Battery was 7.665 V** after the 2026-11-23 reboot; charge before motion
  trials.
- The driver uses a `MultiThreadedExecutor` with intra-process comms. Monitor
  CPU and scan gaps on the Pi 5; if the executor stalls, that is a driver
  issue to measure, not a Nav2 tuning knob.
- The Nav2 parameters are a starting point, not tuned values. Re-tune
  `controller_frequency`, MPPI `batch_size`, inflation and speed limits on
  the real floor.

## Licenses and repo boundaries

- `armpi_mobile_bringup` is MIT.
- The LS-S1 LSLIDAR driver is Apache-2.0. Keep its source in the separate
  `/home/ubuntu/lslidar_ws` workspace and do not vendor it into this repo
  without a THIRD_PARTY_NOTICES entry.
- Do not use the older GPLv3 `M10/M10-PHY_V1.0` driver branch for this
  project.
- RTAB-Map and Nav2 carry their own licenses; see their packages.

## Relationship to the rest of the repo

This package is intentionally isolated: it does not modify
`armpi_ultra_description`, the MoveIt config, `armpi_safety` or the perception
repos. The mobile-base TF and URDF are additive and unused by the live arm
stack. Change the robot's live `robot_state_publisher` only after the mobile
model is measured and reviewed.

The files are uncommitted drafts; review them before committing.
