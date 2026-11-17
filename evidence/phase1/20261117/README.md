# Phase 1 Hardware Baseline — 2026-11-17

> Scope: live ArmPi Ultra over direct Ethernet. Read-only queries plus the
> already-reported supervised `reset_servo` motion. No calibration, grasp
> execution or automated trials. Camera RGB/depth streams were verified; the
> IMU did not publish.

## Environment

| Item | Value |
|---|---|
| Robot | Raspberry Pi 5 Model B Rev. 1.1 |
| OS / kernel | Debian 12 (bookworm), 6.6.20+rpt-rpi-2712, aarch64 |
| Container | `ArmPiUltra`, ROS 2 Humble |
| RMW / domain | `rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=0` |
| Robot / laptop link | SSH baseline over direct Ethernet (`eth0 10.42.0.218/24` ↔ laptop `10.42.0.1/24`); ROS 2 DDS over WiFi (`wlan0 10.51.128.226/23` ↔ `wlp4s0 10.51.128.246/23`) |
| Camera | Deptrum Aurora 930, serial `HY400523001036827G00179`, firmware 2.0.8, SDK 1.1.19 |
| Laptop MoveIt | Humble Docker container, `real_moveit.launch.py` |

## Results

| Area | Result | Evidence |
|---|---|---|
| Host | 7.9 GiB RAM, 5.8 GiB free; root 95% used, ≈2.9 GiB free; 48.8 °C; `throttled=0x0` | `baseline_console_20261117.txt` |
| USB | Aurora 930 on Bus 003 at **480 Mbps (USB 2.0)**; CH340 `/dev/rrc → ttyUSB0`; RTL8188GU Wi-Fi adapter `0bda:1a2b` | `baseline_console_20261117.txt` |
| ROS graph | one `/controller_manager`, one `/servo_manager`, one arm action server + one MoveIt client | `baseline_console_20261117.txt` |
| `joint_states` | ~20 Hz; boot pose `joint1=+0.8378`, others 0; reset pose all 0; `r_joint=-0.8378` (open) | `baseline_console_20261117.txt`, `pose_snapshot_20261117.txt` |
| `servo_states` | boot ID 6 = 700, others 500; reset all 500; goal/error 0; `voltage=0` | `baseline_console_20261117.txt`, `pose_snapshot_20261117.txt` |
| Battery | 7704 mV after reboot, 7651 mV latest; earlier 7385 mV | `baseline_console_20261117.txt`, `pose_snapshot_20261117.txt` |
| TF | `base_link → rgb_camera_link` / `depth_camera_link` resolve; **`camera_optical_frame` does not exist** | `tf_camera_20261117.txt` |
| Camera RGB | 8.0 Hz **in-container** (undercounts; **~12 Hz cross-host**, 2026-11-17 Session 4), 640×400, `bgr8`, frame `rgb_camera_link` | `camera_stream_20261117.txt` |
| Camera depth | 8.2 Hz in-container (not re-measured cross-host), 640×400, `mono16`, frame `depth_camera_link`; values mm; median 2257 mm; 23.11% zeros | `camera_stream_20261117.txt` |
| RGB intrinsics | fx 418.054, fy 419.100, cx 308.783, cy 187.134; D 5-term `plumb_bob` | `camera_stream_20261117.txt` |
| Depth intrinsics | fx 423.023, fy 423.829, cx 320.096, cy 199.904; D 5-term `plumb_bob` | `camera_stream_20261117.txt` |
| IMU | `/ros_robot_controller/imu_raw` has a publisher but no message in 5–12 s | `camera_retry_20261117.txt` |
| Real image | 640×400 `bgr8` frame saved; current view is a white surface with dark edges, not the work surface | `rgb_frame_20261117.png` |

## Incidents and fixes

0. **Camera-rate correction (2026-11-17 Session 4).** The 8.0 Hz RGB number in
   this bundle came from an in-container subscriber. A cross-host laptop
   subscriber measured 11.83 Hz and `ros2 topic hz` ~12.0 Hz for the same
   topic, while an in-container `ros2 topic hz` measured ~7.0 Hz. Local DDS
   delivery drops/queues large images; the camera publishes at about 12 Hz.
   The raw `camera_stream_20261117.txt` is unchanged; see
   [`docs/phase3/HOST_BENCHMARK.md`](../../../docs/phase3/HOST_BENCHMARK.md).

1. **Camera open failure before the restart.** The first `vision.launch` at
   09:24 failed with `open device failed ret=0x21003` (error 135171) and the
   node exited. After the restart, the same node opened the device and
   streamed RGB/depth. Root cause is unresolved; treat it as a transient
   device/USB state.
2. **Camera launch reaped.** A `nohup ros2 launch ... &` inside `docker exec`
   was reaped when the exec session ended; `aurora930_node` became a zombie.
   The persistent form is `docker exec -d ... /bin/zsh -c '... exec ros2 launch ...'`.
3. **Wi-Fi kernel oops.** A stale command loaded `rtw88_8821cu` for a Realtek
   RTL8188GU adapter in CDROM mode (`0bda:1a2b`); the kernel oops wedged USB,
   `lsusb` hung, and a soft reboot stuck in shutdown. A restart
   recovered it. Do not load `rtw88_8821cu` on this robot.
4. **Direct link lost at the end.** The laptop NIC `enx00e04ca373c7` reported
   `Link detected: no` after the capture; the robot remained reachable over
   WiFi `10.51.128.226`, and the camera was restarted over WiFi. The baseline
   above was collected while the direct SSH link was up.

## Files

| File | Content |
|---|---|
| `baseline_console_20261117.txt` | host, USB, ROS env, nodes, topics, joint/servo/battery, TF |
| `camera_retry_20261117.txt` | first camera launch log, RGB camera_info, IMU retry |
| `camera_stream_20261117.txt` | final RGB/depth rates, depth stats, RGB/depth camera_info |
| `tf_camera_20261117.txt` | `base_link → rgb/depth/optical` transforms and `camera_optical_frame` negative control |
| `pose_snapshot_20261117.txt` | reset-pose joint states, servo states, battery |
| `rgb_frame_20261117.png` | one real 640×400 RGB frame |
| `vision_current.log` | camera driver log from the persistent launch |
| `recovery.log` | wait-for-robot and capture sequence |

## Limitations

- `joint_states` and `servo_states` are cached commands, not measured encoder
  feedback. No mechanical joint limits were measured.
- The camera intrinsics are driver-reported, not independently calibrated.
- The camera `link4 → camera` hand-eye extrinsic is the vendor static TF, not a
  measured calibration.
- The TF chain treats RGB and depth as coincident; the driver-reported
  IR-to-RGB offset (~9.86 mm x) is not represented in TF.
- Depth registration and point-cloud accuracy were not validated.
- The IMU is not publishing.
- The camera runs on USB 2.0; the observed cross-host rate is ~12 Hz, adequate
  for static AprilTag localization but not for high-rate servoing. The
  in-container 8 Hz measurement undercounts.

## Reproduce

```bash
# WiFi (DDS + SSH); current path after the direct link dropped
ssh -i ~/.ssh/id_ed25519_armpi pi@10.51.128.226
# direct Ethernet fallback when the cable is connected
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218
sudo docker exec -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; ROS2CLI_NO_DAEMON=1 ros2 node list'

# persistent camera launch
sudo docker exec -d -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; exec ros2 launch bringup vision.launch.py > /tmp/vision.log 2>&1'

# stream check (inside the container)
sudo docker exec -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; python3 /tmp/camera_stream_check.py --seconds 5'
```

## Safety

- The robot boots with `joint1 ≈ +0.8378` (servo 700); keep the workspace clear.
- `reset_servo` moves only joint1 back to 0 in the current configuration.
- Do not run `demo.launch.py` / `mock_components` on the real robot's domain.
- Do not load `rtw88_8821cu`; use the direct Ethernet link.
- Charge the battery before long trials; 7.651 V is not a full pack.
