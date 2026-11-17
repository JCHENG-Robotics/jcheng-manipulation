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
| Gripper | `r_joint`, servo ID 1; live open at `r_joint ≈ -0.8378 rad` | `/joint_states`, `servo_states` | `[V]` |
| Camera | Deptrum Aurora 930; USB `3251:1930`; serial `HY400523001036827G00179`; firmware 2.0.8; SDK 1.1.19 | `aurora930_node` log | `[V]` |
| Camera link | **USB 2.0, 480 Mbps** (Bus 003, Port 1) | `lsusb -t` | `[V]`; bandwidth ceiling |
| Camera streams | RGB 8.0 Hz, 640×400 `bgr8`; depth 8.2 Hz, 640×400 `mono16` (mm) | `camera_stream_check.py` | `[V]` |
| Camera frames | RGB `rgb_camera_link`; depth `depth_camera_link`; no `camera_optical_frame` | `camera_info`, TF | `[V]` |
| Camera calibration | RGB/depth intrinsics read from the driver; `link4 → camera` hand-eye extrinsic **not measured** | `camera_info`, TF | `[V]` intrinsics, `[U]` extrinsic |
| Base | Mecanum chassis; `CHASSIS_TYPE=Mecanum` | container env | `[S]` config, `[U]` motion |
| E-stop / power cut | physical stop not verified | — | `[U]` |
| Battery | 2S pack; 7651 mV latest (7704 mV after reboot, 7385 mV earlier) | `/ros_robot_controller/battery` | `[V]` |
| Storage | root filesystem 95% used, ≈2.9 GiB free | `df -h /` | `[V]`; boot/log risk |
| Thermal | 48.8 °C after boot, `throttled=0x0` | `vcgencmd` | `[V]` |
| Network | `eth0 10.42.0.218/24`; `wlan0 10.51.128.226/23` Wi-Fi client | `ip -brief addr` | `[V]` |
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
- **Battery.** 7.651 V is usable but below the earlier 7.704 V. Charge before
  long motion trials and avoid charging while the arm is moving.
- **Direct link.** The laptop's direct NIC showed **no link** at the end of the
  session (cable/power), so the robot was offline after the capture. The data
  above was collected while the link was up.
- **Storage.** 95% root usage is a boot/fsck and logging risk. Free space before
  recording bags.

## To collect

- Photos of the arm, controller, camera mount, base and stop.
- Servo EEPROM offsets (read-only tool) and measured joint stops.
- A verified physical power cut / E-stop.
- Battery/charger specs and brown-out behavior.
- Camera mount rigidity and a real `link4 → camera` extrinsic.
