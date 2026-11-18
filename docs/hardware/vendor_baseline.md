# Vendor Baseline — read-only reference

> The vendor stack is **not** copied into this repository. It runs inside the
> robot container and is also archived privately. Paths below are reference-only.

## Robot-side vendor stack (live, 2026-11-17)

| Item | Path / value | Status |
|---|---|---|
| Container | `ArmPiUltra` (Humble) | `[V]` |
| Vendor workspace | `/home/ubuntu/ros2_ws` inside the container | `[V]` |
| Core launch | `bringup bringup.launch.py` via `start_node.service` | `[V]` |
| Camera launch | `bringup vision.launch.py` | `[V]` |
| Servo controller | `driver/servo_controller` | `[V]` |
| Robot controller | `driver/ros_robot_controller` (CH340 `/dev/rrc`) | `[V]` |
| Kinematics | vendor `kinematics` package + aarch64 `.so` | `[S]` |
| Camera driver | `third_party_ros2/third_party_ws/install/deptrum-ros-driver-aurora930` | `[V]` |
| Camera firmware / SDK | 2.0.8 / 1.1.19 | `[V]` |
| Camera serial | `HY400523001036827G00179` | `[V]` |

## Vendor configuration files

| File | Content | Status |
|---|---|---|
| `driver/servo_controller/config/servo_controller.yaml` | joint1 ID 6, joint2 ID 5, joint3 ID 4 (min 1000 / max 500 inverted), joint4 ID 3, joint5 ID 2, r_joint ID 1 (`init: 700`) | `[S]` |
| Camera `transform.yaml` | `extristric`, `plane`, `white_area_pose_cam`, `white_area_pose_world`, `corners`; vendor **table/white-area** calibration, **not** a hand-eye matrix. Decoded in [vendor_calibration_baseline.md](vendor_calibration_baseline.md) | `[S]` |
| Camera `camera_info.yaml` | `hand2cam_tf_matrix`; vendor end-effector←camera matrix. The name is misleading: this is **not** camera intrinsics. The shipped matrix is **stale/different for the current mount** (89.6 mm from the validated link5-relative result) | `[S]` + static analysis |

The live RGB/depth intrinsics come from the camera driver, not from
`camera_info.yaml`; see `docs/hardware/camera_mount.md`.

## Private archive

| Item | Value |
|---|---|
| Private workspace | `armpi_ultra_ws_clean/` (gitignored; 12 retained vendor packages, aarch64 IK/FK `.so`) |
| Archive | `~/backups/2026-11-16/armpi_ultra_ws_clean_20261115.tar.gz` |
| SHA256 | `76b5a4cd379f32845194ccc51c9b4ea357f8f87fc89b8cf823ba9bea811ebfc6` |
| Manifest | `~/backups/2026-11-16/armpi_ultra_ws_clean_20261115.tar.gz.MANIFEST.md` |
| Vendor SDK hash | `369b11dd…` (reference) |

The workspace has no declared license, so it must stay out of the public
GitHub repository.

## Private modifications made for the real-robot stack

These changes are in the private vendor workspace (robot container source and
install), not in the public repository:

1. `driver/servo_controller/launch/servo_controller.launch.py`: removed
   `name='controller_manager'`. That remap also renamed the internal
   `ServoManager` node, producing two `/controller_manager` nodes and two
   action servers per action.
2. Robot `.zshrc`: added `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
   `CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml`.
3. `/home/ubuntu/cyclonedds.xml`: bind CycloneDDS to the active link. Since
   2026-11-18 this is **`wlan0`** (office WiFi); the wired config is backed up
   as `cyclonedds.xml.bak_eth0_20261118`. A `wlan0: does not match an available
   interface` failure means the interface is down — bring WiFi up before
   restarting the core. Change both ends together.
4. Restarted `start_node.service` and verified one `/controller_manager`, one
   `/servo_manager`, one action server and one MoveIt client.
5. `driver/controller/controller/mecanum.py` (chassis node): **hardened fork**
   with a `/cmd_vel` watchdog — `cmd_vel_timeout` (default 0.5 s) and
   `watchdog_rate` (default 20 Hz) parameters, zero duty published when a
   non-zero command goes stale, and zero duty on shutdown. Vendor original
   sha256 `9c54a5d5…`; deployed file sha256 `c599517a…`. The change is also in
   the private workspace copy; diff:
   [patches/controller_mecanum_watchdog.patch](patches/controller_mecanum_watchdog.patch).

## Known vendor issues seen live

| Issue | Detail | Status |
|---|---|---|
| Cached joint state | `joint_states` / `servo_states` echo commands; `servo_states.voltage` is always 0 | `[V]` |
| Camera open transient | `aurora930_node` failed with `0x21003` before a restart; opened after it | `[V]` observed |
| IMU quiet | `/ros_robot_controller/imu_raw` has a publisher but no messages in 5–12 s | `[U]` |
| Wrong Wi-Fi module | The Realtek RTL8188GU adapter (`0bda:1a2b`, CDROM mode) is not an `rtw88_8821cu` device; loading that module crashed the kernel and wedged USB | `[V]` |
| Root disk | 87% used, ≈6.8 GiB free (2026-11-17 evening); boot/fsck and logging risk | `[V]` |
| Single `/dev/rrc` owner | Multiple paths can open the CH340; no single-owner guarantee | `[S]` |

## Version caution

Two vendor xacro copies exist:

- `3 功能包文件/...` (2025-07-02, SHA `158224…`) — used by the imported sim
  package.
- `ROS2/Armpi_ultra-main/...` (2026-04-02, SHA `7ce1d0…`) — newer; only the
  `base_link` collision geometry differs.

The robot's live vendor `armpi_ultra_description` publishes
`camera_connet_link` but **not** the project's `camera_link` /
`camera_optical_frame`. The public repository's `armpi_ultra_description` adds
those project frames for the model/mock path. Record which model a test uses;
do not silently switch.

## Separation rule

- Vendor code: read-only reference, not modified, not copied into the public
  repository.
- Personal code: `ros2_ws/src/armpi_*`, `armpi-debug-studio/`, `evidence/`,
  `docs/`, `tools/`.
- If a vendor fix is required, record the diff here and get approval before
  touching the original.
