# Hardware Inventory — ArmPi Ultra

> Status: **LIVE BASELINE CAPTURED (2026-11-17)**. Read-only queries; the only
> commanded motion was the supervised `reset_servo` test. `[V]` = verified by a
> live command/log; `[S]` = vendor source/config; `[U]` = not measured or not
> operational.

| Item | Value | Evidence | Status |
|---|---|---|---|
| Arm model | ArmPi Ultra (5-DoF + parallel gripper) | vendor docs | `[S]` |
| Main compute | Raspberry Pi 5 Model B Rev. 1.1 (BCM2712) | `uname -a`, kernel log | `[V]` |
| OS / kernel | Debian 12 (bookworm), 6.6.20+rpt-rpi-2712, aarch64 | `/etc/os-release`, `uname` | `[V]` |
| Controller board | STM32 (vendor firmware) | vendor source | `[S]` |
| Serial link | `/dev/rrc` (CH340) → `/dev/ttyUSB0` | `ls -l /dev/rrc` | `[V]` |
| Bus servos | IDs 1–6; live positions 500 at the reset pose | `/controller_manager/servo_states` | `[V]` |
| Gripper | `r_joint`, servo ID 1; **measured 2026-11-18**: increasing `r_joint` closes; range `-0.8378 … 0.0 rad`; servo register 502 (open) … 642 (closed) | action + `bus_servo/get_state` | `[V]` |
| Camera | Deptrum Aurora 930; USB `3251:1930`; serial `HY400523001036827G00179`; firmware 2.0.8; SDK 1.1.19 | `aurora930_node` log | `[V]` |
| Camera link | **USB 2.0, 480 Mbps** (Bus 003, Port 1) | `lsusb -t` | `[V]`; bandwidth ceiling |
| Camera streams | RGB **~12 Hz** cross-host, 640×400 `bgr8` (2026-11-17 Session 4); in-container subscribers undercount to ~8 Hz. Depth measured 8.2 Hz in-container (not re-measured cross-host). | `host_benchmark.py`, `ros2 topic hz` | `[V]` |
| Camera frames | RGB `rgb_camera_link`; depth `depth_camera_link`; measured optical `camera_optical_calibrated` (project-published); no live `camera_optical_frame` | `camera_info`, TF | `[V]` |
| Camera calibration | RGB/depth intrinsics read from the driver; `link4 → camera` hand-eye **validated** (2.94 mm / 0.39°); body→optical published as `camera_optical_calibrated` | `camera_info`, TF, `evidence/calibration/20261117/` | `[V]` |
| Base | Mecanum chassis; `CHASSIS_TYPE=Mecanum` | container env | `[S]` config, `[U]` motion |
| E-stop / power cut | **No electronic e-stop or brake.** Main-power cut verified 2026-11-18: the arm falls immediately (joints 2–5 lose holding torque); Pi reboots automatically in ~89 s and `joint1` re-homes to +0.8378 | operator observation, ping/SSH | `[V]` |
| Battery | 2S pack; 7704 mV after the 2026-11-18 reboot | `/ros_robot_controller/battery` | `[V]` |
| Storage | root filesystem 84.7% used, ≈6.6 GiB free (2026-11-17 Session 4) | `df -h /` | `[V]`; boot/log risk |
| Thermal | 48.8 °C after boot, `throttled=0x0` | `vcgencmd` | `[V]` |
| Network | `eth0 10.42.0.218/24` (SSH + DDS, current); `wlan0 10.51.128.226/23` (fallback; office WiFi) | `ip -brief addr` | `[V]` |
| Wi-Fi USB adapter | Realtek RTL8188GU `0bda:1a2b` in CDROM mode; `rtw88_8821cu` is the wrong module and crashed the kernel | `lsusb`, `dmesg` | `[V]` |

## Notes

- **Camera open failure and recovery.** Before the restart,
  `aurora930_node` found the device but failed to open it
  (`open device failed ret=0x21003`, error 135171). After the restart
  the same node opened the device and streamed RGB and depth. Treat the failure
  as a transient USB/device-state issue, not a proven hardware fault.
- **Launch persistence.** Start the camera with
  `docker exec -d ... /bin/zsh -c '... exec ros2 launch bringup vision.launch.py'`.
  A `nohup ... &` inside `docker exec` was reaped when the exec session ended,
  leaving a zombie `aurora930_node`; use the `docker exec -d ... exec` form.
- **Boot pose moves joint1.** The vendor startup init sets servo ID 6 to 700,
  i.e. `joint1 ≈ +0.8378 rad` (48°). The arm was at `joint1 ≈ 0` after the
  supervised `reset_servo` test. Keep the workspace clear when the robot boots.
- **Cached commands, not feedback.** `/joint_states` and
  `/controller_manager/servo_states` echo the last commanded values; the servo
  `voltage` field is 0 (not populated). Do not treat them as encoder feedback.
  The separate `/ros_robot_controller/bus_servo/get_state` service reads the
  servo position register and is used by the hand-eye calibration; see
  [`measured_joint_limits.md`](measured_joint_limits.md).
- **Battery.** 7.651 V is usable but below the earlier 7.704 V. Charge before
  long motion trials and avoid charging while the arm is moving.
- **DDS link.** The container's CycloneDDS is bound to `eth0` (since
  2026-11-17 evening); the laptop config binds `enx00e04ca373c7`. The office
  WiFi (`wlan0` / `wlp4s0`) is the fallback and requires switching both
  configs together.
- **Storage.** 87% root usage (≈6.8 GiB free on 2026-11-17 evening) is still a
  boot/fsck and logging risk. Free space before recording bags.

## To collect

- Photos of the arm, controller, camera mount and base.
- Servo EEPROM offsets (read-only tool) and **arm-joint** mechanical stops
  (the gripper stops are measured; arm joints 1–5 are still `[U]`).
- Battery/charger specs and brown-out behavior.
- A physical fiducial at a surveyed `base_link` position (for an independent
  check of the validated optical extrinsic).
