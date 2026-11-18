# Humble MoveIt/RViz on the laptop (real ArmPi)

> Goal: run MoveIt 2 + RViz in **ROS 2 Humble** on the laptop, matching the
> Humble robot. This avoids the Jazzy ↔ Humble DDS discovery problem that
> leaves RViz with no `/joint_states`/TF or no action server.
>
> **Verified end-to-end on 2026-11-16**: one real `controller_manager`, one
> arm/gripper action server, MoveIt client connected, `/joint_states` 20 Hz,
> RViz loaded the `armpi_ultra` model.
>
> **Wired link is the only DDS path since 2026-11-18.** The robot runs a
> default-deny inbound firewall (`docs/hardware/network_access.md`), so this
> container must bind `enx00e04ca373c7` (10.42.0.1) and the robot `eth0`
> (10.42.0.218). A WiFi-bound container can no longer discover the robot.
> The 2026-11-17 office-WiFi verification is historical.

## Verified working configuration

| Part | Configuration |
|---|---|
| Laptop | Ubuntu 24.04, Docker 29.9.3, image `armpi-humble-moveit` |
| Laptop RMW | `rmw_cyclonedds_cpp`, `cyclonedds-laptop.xml` on `enx00e04ca373c7` (direct Ethernet 10.42.0.1) |
| Robot | existing Humble `armpi_ultra_ws_clean`, `rmw_cyclonedds_cpp` |
| Robot network | `eth0` only in `cyclonedds.xml`; robot `10.42.0.218`, laptop `10.42.0.1` |
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
        <NetworkInterface name="eth0" priority="1" multicast="true"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF

grep -q RMW_IMPLEMENTATION ~/.zshrc || \
  printf '\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nexport CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml\n' \
  >> ~/.zshrc
```

To use the office WiFi for internet access (not DDS), switch the onboard
`wlan0` from AP mode to a client profile (this disables the `HW-5401EF08` AP):

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

Keep **only `eth0`** in the robot XML. Listing a second interface breaks
participant creation when that interface has no IPv4 address (`rcl node's rmw
handle is invalid`), and a WiFi interface is no longer reachable after the
inbound lockdown.

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
`CYCLONEDDS_URI=/config/cyclonedds-laptop.xml` (mounted outside the workspace
bind; see the 2026-11-18 pitfalls below) and `ROS2CLI_NO_DAEMON=1`.

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

## Named states on the real robot

The SRDF named states are loaded as `robot_description_semantic`. RViz lists
them under **MotionPlanning → States**, and **Plan & Execute** sends a
`FollowJointTrajectory` goal through `move_group` to the robot's
`arm_controller`. No extra node is needed; `real_moveit.launch.py` loads the
SRDF from this package explicitly.

| State | Group | Meaning |
|---|---|---|
| `reset_servo` | `arm` | all arm joints at 0; smallest first test |
| `init_pose` | `arm` | vendor folded pose |
| `camera_forward` | `arm` | camera optical +z horizontal forward; camera at x≈0.154 m, z≈0.282 m |

Procedure:

1. Start the robot bringup and the Humble container as above.
2. Launch `real_moveit.launch.py` (not `demo.launch.py`).
3. In RViz set Fixed Frame `base_link`, Planning Group `arm`, and set velocity
   and acceleration scaling to **0.05** on the **Planning** tab.
4. Select `camera_forward` as the goal: use the **Planning → Goal State**
   dropdown, or open the **States** tab and click **Set as Goal**.
5. Click **Plan**, inspect the green trajectory, then **Execute**.
6. Keep the arm clear, keep the power cut within reach, and do not run
   unattended.

`reset_servo` from the current pose is the smallest test; `camera_forward` is a
larger forward reach. The `arm` group does not move `r_joint`, so the gripper
stays where it is. The `open`/`close` gripper states are model-only until the
`r_joint` sign convention is measured (see
`docs/hardware/measured_joint_limits.md`).

The documented workspace is built with `--symlink-install`, so an SRDF edit is
picked up after relaunching `move_group`. Without symlink install, rebuild with
`colcon build --packages-select armpi_ultra_moveit_config` first.

For a plan-only check without RViz, copy
`evidence/armpi_moveit/real_camera_forward_20261117/plan_named_state_check.py`
into the container and run `python3 /tmp/plan_named_state_check.py <state>`
after sourcing the workspace. It calls `/plan_kinematic_path` and never sends an
execute goal.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `I have no name!` / `groups: cannot find name for group ID 984` | Old image lacks UID 1000. Rebuild with the current Dockerfile; the new image has `user`. The 984 warning is cosmetic. |
| `ros2 node list` empty; `unknown tag 'rclpy.type_hash.TypeHash'` | The Humble CLI connected to the host Jazzy `ros2` daemon. Stop it with Jazzy `ros2 daemon stop`, or use `ROS2CLI_NO_DAEMON=1` (the script sets it). |
| Two action servers | The robot's `servo_controller.launch.py` still has `name='controller_manager'`; remove it and restart `start_node.service`. A Jazzy/mock demo in the same graph also causes duplicates. |
| Plan Failed; `Start state appears to be in collision ... l_link ... r_in_link` | Vendor finger meshes overlap. Ensure the SRDF finger-pair `disable_collisions` entries are present and the MoveIt container was restarted after rebuilding. |
| RViz shows no arm | Check `/joint_states` and TF in the container; Fixed Frame `base_link`; CycloneDDS interfaces (`enx00e04ca373c7` laptop, `eth0` robot). |
| `does not match an available interface` or no robot nodes | The container was started before `cyclonedds-laptop.xml` was changed. A bind-mounted file keeps the old inode, so restart the container with `run_humble.sh`; editing the host file in place is not enough. Also check that the file is non-empty (`wc -c /config/cyclonedds-laptop.xml`) and that the interface name matches the current link. |
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

## Wired-link and DDS pitfalls (2026-11-18)

These three issues each look like "no robot nodes" in the container. Check them
in order:

1. **NetworkManager `shared` mode drops inbound UDP.** With
   `ipv4.method shared` on the laptop's `armpi-direct` connection, robot→laptop
   UDP was **0/300** while laptop→robot was 300/300, so DDS discovery never
   completed. Use a plain static link:

   ```bash
   nmcli connection modify armpi-direct ipv4.method manual \
       ipv4.addresses 10.42.0.1/24 ipv4.gateway "" ipv4.dns "" \
       ipv4.never-default yes
   nmcli connection modify "Wired connection 1" ipv4.method manual \
       ipv4.addresses 10.42.0.218/24 ipv4.gateway "" ipv4.dns ""
   ```

2. **EEE causes intermittent loss.** The RTL8153 USB NIC negotiated
   Energy-Efficient Ethernet with the Pi: ping 23/30, SSH banner timeouts and
   DDS flapping. Disable EEE on the laptop (the robot's older NetworkManager
   has no EEE property; one side is enough):

   ```bash
   nmcli connection modify armpi-direct ethtool.eee-enabled off
   nmcli connection up armpi-direct
   ```

   After this: ping 30/30, UDP 300/300.

3. **A file bind mount under `/workspace` can resolve to a 0-byte
   placeholder.** `-v .../cyclonedds-laptop.xml:/workspace/cyclonedds-laptop.xml`
   left an empty root-owned file inside the workspace bind mount; CycloneDDS
   then had no interface and bound elsewhere. `run_humble.sh` now mounts the
   config at `/config/cyclonedds-laptop.xml` (outside the workspace bind) and
   uses `CYCLONEDDS_URI=file:///config/cyclonedds-laptop.xml`. If you start the
   container by hand, do the same and verify the file is non-empty:

   ```bash
   wc -c /config/cyclonedds-laptop.xml
   ```

