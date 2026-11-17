# Humble MoveIt/RViz on the laptop (real ArmPi)

> Goal: run MoveIt 2 + RViz in **ROS 2 Humble** on the laptop, matching the
> Humble robot. This avoids the Jazzy ↔ Humble DDS discovery problem that
> leaves RViz with no `/joint_states`/TF or no action server.
>
> **Verified end-to-end on 2026-11-16**: one real `controller_manager`, one
> arm/gripper action server, MoveIt client connected, `/joint_states` 20 Hz,
> RViz loaded the `armpi_ultra` model.
>
> **WiFi link verified 2026-11-17**: the robot joins the open `tjyx_office`
> network with its onboard `wlan0` (AP mode disabled, profile
> `tjyx_office-robot`, DHCP `10.51.128.226`); the laptop uses `wlp4s0`
> (`10.51.128.246`). Same 20 Hz `/joint_states`, 1 action server, TF OK.

## Verified working configuration

| Part | Configuration |
|---|---|
| Laptop | Ubuntu 24.04, Docker 29.9.3, image `armpi-humble-moveit` |
| Laptop RMW | `rmw_cyclonedds_cpp`, `cyclonedds-laptop.xml` on `wlp4s0` (office WiFi) |
| Robot | existing Humble `armpi_ultra_ws_clean`, `rmw_cyclonedds_cpp` |
| Robot network | `wlan0` (office WiFi) only in `cyclonedds.xml`; robot `10.51.128.226`, laptop `10.51.128.246` |
| MoveIt | `real_moveit.launch.py`: `move_group` + RViz only, no mock controllers |
| Verified | 1 action server arm + 1 gripper; 1 MoveIt client; `/joint_states` 20 Hz |

## Why not the Jazzy demo launch

`demo.launch.py` with `mock_components` starts laptop-local mock controllers,
`robot_state_publisher` and a static transform. When the real robot is in the
same graph, this creates duplicate action servers under
`/arm_controller/follow_joint_trajectory`, duplicate TF and duplicate
`/joint_states`. MoveIt can then execute the same goal on the mock and real
controllers. For real hardware, use `real_moveit.launch.py` instead: it starts
only `move_group` + RViz and relies on the robot for state, TF and actions.

## Robot-side setup (once)

The robot's Humble image already has `rmw_cyclonedds_cpp`.

```bash
# inside the robot container (user ubuntu)
cat > /home/ubuntu/cyclonedds.xml <<'EOF'
<CycloneDDS>
  <Domain Id="0">
    <General>
      <Interfaces>
        <NetworkInterface name="wlan0" priority="1" multicast="true"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF

grep -q RMW_IMPLEMENTATION ~/.zshrc || \
  printf '\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nexport CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml\n' \
  >> ~/.zshrc
```

To use the office WiFi, switch the onboard `wlan0` from AP mode to a client
profile (this disables the `HW-5401EF08` AP):

```bash
sudo nmcli connection modify HW-5401EF08 connection.autoconnect no
sudo nmcli connection add type wifi ifname wlan0 con-name tjyx_office-robot ssid tjyx_office
sudo nmcli connection modify tjyx_office-robot wifi.powersave 2
sudo nmcli connection up tjyx_office-robot ifname wlan0
```

Do **not** use a USB RTL8821CU dongle here: the in-kernel `rtw88_8821cu`
driver scans but every association ends in `authentication timed out`, and
reloading it caused a kernel trace and a hard hang on this Pi. It is
blacklisted via `/etc/modprobe.d/blacklist-rtw88-8821cu.conf`.

Keep **only `wlan0`** in the robot XML. Listing `eth0` as a second interface
breaks participant creation once the cable is unplugged (`rcl node's rmw
handle is invalid`), because a listed interface has no IPv4 address then.

In the private `armpi_ultra_ws_clean` workspace, remove
`name='controller_manager'` from
`src/driver/servo_controller/launch/servo_controller.launch.py`. That launch
remap also renames the internal `ServoManager` node, producing two
`/controller_manager` nodes and making every action server appear twice.

Restart the core stack:

```bash
sudo systemctl stop start_node.service
sudo docker exec -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c '~/.stop_ros.sh'
sudo systemctl start start_node.service
```

The MoveIt model also disables the vendor finger-finger collision pairs in
`armpi_ultra_moveit_config/config/armpi_ultra.srdf` (`l_link`/`r_in_link`, etc.).
The meshes overlap at some `r_joint` values, which otherwise makes the planner
report the start state as self-colliding.

## Laptop setup

### 1. Docker (once, needs sudo)

```bash
cd ~/jcheng/jcheng-manipulation
bash tools/humble_moveit/install_docker.sh
```

Then `newgrp docker` or log out/in once. All later commands can use:

```bash
sg docker -c "..."
```

### 2. Build the Humble image

```bash
cd ~/jcheng/jcheng-manipulation
sg docker -c "docker build -t armpi-humble-moveit \
  -f tools/humble_moveit/Dockerfile tools/humble_moveit"
```

The base is `ros:humble-ros-base`; the Dockerfile adds MoveIt, RViz,
`moveit_configs_utils`, the Pilz planner, CycloneDDS and a UID 1000 `user`
(matching the host) so the interactive container has a normal prompt.

### 3. Prepare the Humble workspace

```bash
bash tools/humble_moveit/setup_humble_ws.sh
```

This symlinks `armpi_ultra_description` and `armpi_ultra_moveit_config` into
`~/jcheng/humble_ws/src`. `run_humble.sh` mounts the repo source at the same
absolute path inside the container, so the absolute symlinks resolve.

### 4. Start the container

```bash
sg docker -c "bash tools/humble_moveit/run_humble.sh"
```

The script uses `--net=host`, mounts the X11 socket and the workspace, sets
`ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
`CYCLONEDDS_URI=/workspace/cyclonedds-laptop.xml` and
`ROS2CLI_NO_DAEMON=1`.

### 5. Build and launch inside the container

```bash
source /opt/ros/humble/setup.bash
cd /workspace
colcon build --symlink-install \
  --packages-select armpi_ultra_description armpi_ultra_moveit_config
source install/setup.bash
ros2 launch armpi_ultra_moveit_config real_moveit.launch.py
```

## Verify before moving

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

ros2 node list
ros2 topic hz /joint_states
ros2 run tf2_ros tf2_echo base_link link1
ros2 action info /arm_controller/follow_joint_trajectory
```

Expected:

- `/controller_manager` and `/servo_manager` each appear **once**;
- `/joint_states` is about **20 Hz** with no large gaps;
- `tf2_echo` returns transforms;
- exactly **one** action server and, once `move_group` is up, one client
  (`/moveit_simple_controller_manager`).

## First real motion

- In RViz: Fixed Frame `base_link`, Planning Group `arm`.
- From the current vertical pose (`joint1≈0.84`, `joint2..5=0`), `reset_servo`
  (all-zero arm) is a small joint1-only test; use it first.
- `init_pose` folds the arm and is a larger move; try it after `reset_servo`.
- Set velocity and acceleration scaling to **0.05**.
- Click **Plan** first, inspect the green trajectory, then **Execute**.
- Keep the arm clear, keep the power cut within reach, and do not run unattended.
- The `arm` group does not move `r_joint`; the gripper stays where it is.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `I have no name!` / `groups: cannot find name for group ID 984` | Old image lacks UID 1000. Rebuild with the current Dockerfile; the new image has `user`. The 984 warning is cosmetic. |
| `ros2 node list` empty; `unknown tag 'rclpy.type_hash.TypeHash'` | The Humble CLI connected to the host Jazzy `ros2` daemon. Stop it with Jazzy `ros2 daemon stop`, or use `ROS2CLI_NO_DAEMON=1` (the script sets it). |
| Two action servers | The robot's `servo_controller.launch.py` still has `name='controller_manager'`; remove it and restart `start_node.service`. A Jazzy/mock demo in the same graph also causes duplicates. |
| Plan Failed; `Start state appears to be in collision ... l_link ... r_in_link` | Vendor finger meshes overlap. Ensure the SRDF finger-pair `disable_collisions` entries are present and the MoveIt container was restarted after rebuilding. |
| RViz shows no arm | Check `/joint_states` and TF in the container; Fixed Frame `base_link`; CycloneDDS interfaces (`wlp4s0` laptop, `wlan0` robot). |
| No robot nodes, but data arrives | Check `--net=host`, `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`; run `ros2 daemon stop` inside the container. |
| Pi load high / SSH stalls | Close Jazzy MoveIt/RViz; the old Jazzy discovery storm overloads the Pi. Use the IPv6 link-local SSH fallback if IPv4 SSH stalls. |
| `Action server /recognize_objects not available`, `No 3D sensor plugin`, `amdgpu drmGetDevice2 failed` | Expected/harmless in this setup. |
| MoveIt plans but robot stutters | Watch `/joint_states` gaps and `/ros_robot_controller/battery`; start at 0.05 scaling; avoid charging while running. |

## Safety notes

- This controls the real robot. The servo controller is open-loop;
  `joint_states` are cached commands, not measured feedback.
- Do not run `demo.launch.py` with `mock_components` while the real robot is in
  the same graph.
- Do not run the Jazzy MoveIt/RViz and this Humble container at the same time on
  the same ROS domain.
- Keep the emergency power cut accessible and never run automated trials
  unattended until the safety checklist is complete.
- `armpi_ultra_ws_clean` is a private vendor-derived workspace; keep it out of
  the public repository.
