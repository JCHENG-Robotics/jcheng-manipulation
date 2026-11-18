# Wireless link switch — 2026-11-18

> Status: **PASS with caveats.** The robot and the laptop now talk over the
> office WiFi (`tjyx_office`): SSH, CycloneDDS discovery, `/joint_states`,
> `/tf` and the gateway HTTP API all work with the wired cable still attached
> as a fallback. The WiFi path jitters more than the cable (TF max gap
> 110 ms), cross-host image topics drop to ~2 Hz, and the SSID is **open**, so
> the ROS graph and the gateway port are exposed to the LAN.

## What changed

| Side | Change |
|---|---|
| Robot `wlan0` | NM profile `tjyx_office-robot` recreated (open SSID), `HW-5401EF08` AP profile `autoconnect no`; DHCP `10.51.128.226/23` |
| Robot DDS | `/home/ubuntu/cyclonedds.xml` bound `eth0` → **`wlan0`**; wired backup `cyclonedds.xml.bak_eth0_20261118` |
| Robot core | `start_node.service` restarted; the arm **did not move** (measured pulses 498/596/831/113/500 before, 498/595/830/111/500 after, ≤2 pulses) |
| Laptop DDS | `~/jcheng/humble_ws/cyclonedds-wired.xml` and `tools/humble_moveit/cyclonedds-laptop.xml` bound `enx00e04ca373c7` → **`wlp4s0`** |
| Laptop stack | `real_moveit.launch.py use_rviz:=true` and `armpi_safety` restarted inside `armpi-humble` |
| Gateway | `ARMPI_GATEWAY_BIND` `10.42.0.218` → **`10.51.128.226`**, service restarted |

## Verification

| Check | Result |
|---|---|
| Laptop → robot ping | 4/4, 0 % loss, 7.0–24.3 ms |
| Robot → laptop ping | 3/3, 0 % loss, 5.9–65.3 ms |
| SSH over WiFi | `ssh -i ~/.ssh/id_ed25519_armpi pi@10.51.128.226` works |
| DDS over WiFi | `/ros_robot_controller/battery` = 8289 mV and `/joint_states` received on the laptop |
| `/tf` rate | 12.5 Hz, min 29 ms / max 110 ms gap, std 27 ms |
| Gateway `/health` | `status ok`, `ros_ready`, `battery_ready`, `joints_ready`, `measured_ready`, `tf_ready` |
| Gateway `/battery` | 8289 mV, 94.5 %, `truth=measured` |
| Camera | restarted after the core restart; `/camera/status` available, 640×400 bgr8, 6.5 fps **inside the container** |
| Camera cross-host | `/depth_cam/rgb/image_raw` on the laptop drops to **~2 Hz** (max gap 1.05 s); depth had no message in 10 s — images are not usable cross-host over WiFi |
| Node list | still fails with the known `empty node name returned by the RMW layer`; topics work |

## Caveats

1. **Camera needs a restart after a core restart, and images stay local.**
   `vision.launch.py` was stopped by `~/.stop_ros.sh` during the core restart;
   it was restarted with the persistent `docker exec -d ... exec` form and
   `/camera/status` is available. Cross-host image DDS is only ~2 Hz over
   WiFi: use the gateway HTTP snapshot (`/camera/snapshot`) or run capture
   tools inside the robot container; do not expect laptop-side image
   subscribers to keep up.
2. **Chassis node not started.** `/cmd_vel` does not exist after the restart;
   the chassis test still needs `ros2 run controller mecanum`.
3. **Open WiFi.** `tjyx_office` has no password. Domain 0 has no DDS
   authentication, so any host on that LAN can see and publish to the graph;
   the gateway is plain HTTP with a token header, so the token can be
   sniffed. Do not expose motion on this link beyond supervised use.
4. **Latency.** WiFi adds jitter (TF max gap 110 ms vs the wired path). Use
   the wired link for timing-sensitive captures; keep the cable as fallback.

## Rollback (wired)

```bash
# robot
sudo docker exec -u ubuntu ArmPiUltra /bin/zsh -c \
  'cp /home/ubuntu/cyclonedds.xml.bak_eth0_20261118 /home/ubuntu/cyclonedds.xml'
sudo systemctl restart start_node.service
# laptop: set enx00e04ca373c7 in ~/jcheng/humble_ws/cyclonedds-wired.xml,
# then restart the MoveIt launch inside armpi-humble
# gateway
sudo sed -i 's/^ARMPI_GATEWAY_BIND=.*/ARMPI_GATEWAY_BIND=10.42.0.218/' \
  /etc/armpi-gateway/gateway.env && sudo systemctl restart armpi-gateway
```
