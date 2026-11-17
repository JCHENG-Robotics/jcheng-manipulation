# Driver Interfaces — live audit

> Status: source audit plus **read-only live queries on 2026-11-17**. No servo
> target was changed by this audit. `joint_states` and `servo_states` are cached
> commands, not measured encoder feedback. The bus-servo `get_state` service is
> a hardware position read and is used by the hand-eye calibration.

## Live interfaces

| Interface | Type | Live observation (2026-11-17) | Status |
|---|---|---|---|
| `/joint_states` | topic | ~20 Hz; names `joint1…joint5`, `r_joint`, `l_joint`, `l_in_joint`, `l_out_joint`, `r_in_joint`, `r_out_joint` | `[V]` |
| `/controller_manager/joint_states` | topic | present; same cached-command semantics | `[V]` |
| `/controller_manager/servo_states` | topic | IDs 1–6; `frame_id: base_link`; at reset all positions 500, goal 0, error 0, voltage 0; at boot ID 6 = 700 | `[V]` |
| `/ros_robot_controller/battery` | topic | `7651` mV latest (7704 mV after reboot) | `[V]` |
| `/ros_robot_controller/imu_raw` | topic | one publisher; no message in 5–12 s with sensor QoS | `[U]` |
| `/arm_controller/follow_joint_trajectory` | action | one server (`/controller_manager`), one client (`/moveit_simple_controller_manager`) | `[V]` |
| `/gripper_controller/follow_joint_trajectory` | action | one server | `[V]` |
| `/servo_controller` | topic | pulse command path | `[S]` |
| `ros_robot_controller/bus_servo/get_state` | service | `[V]` 2026-11-17: read arm IDs 2–6; positions differ from the cached commands by up to 8 pulses (1.9°); used by the hand-eye calibration. Each `get_position` read logs at INFO and can block the single-threaded driver. | `[V]` |
| `/kinematics` services | services | vendor IK/pose services | `[S]` |

## Camera interfaces (when `vision.launch.py` runs)

| Topic | Type | Live observation | Status |
|---|---|---|---|
| `/depth_cam/rgb/image_raw` | `sensor_msgs/Image` | **~12 Hz cross-host**; in-container subscribers undercount to ~8 Hz; 640×400, `bgr8`, frame `rgb_camera_link` | `[V]` |
| `/depth_cam/rgb/camera_info` | `sensor_msgs/CameraInfo` | 640×400, `plumb_bob`, frame `rgb_camera_link` | `[V]` |
| `/depth_cam/depth/image_raw` | `sensor_msgs/Image` | 8.2 Hz in-container, 640×400, `mono16`, frame `depth_camera_link`; values in mm | `[V]` |
| `/depth_cam/depth/camera_info` | `sensor_msgs/CameraInfo` | 640×400, `plumb_bob`, frame `depth_camera_link` | `[V]` |
| `/depth_cam/depth_registered/points` | `sensor_msgs/PointCloud2` | advertised by the driver; not separately validated | `[U]` |
| `/depth_cam/ir/image_raw` | `sensor_msgs/Image` | advertised by the driver; not separately validated | `[U]` |

Camera launch uses `bringup vision.launch.py` and the Deptrum
`deptrum-ros-driver-aurora930` node. Topics use sensor-data QoS (best effort);
`ros2 topic echo` needs `--qos-profile sensor_data`.

## Known vendor defects (source-level, from G0)

- `get_torque_state` calls a non-existent method → `AttributeError`.
- `pub_button_data` uses an unimported `PacketReportKeyEvents` → `NameError`.
- `/joint_controller` callback parameter type is wrong.
- SDK reads use `queue.get(block=True)` with no timeout; the single-threaded
  executor can freeze.
- Multiple paths can open `/dev/rrc`; no single-owner guarantee.
- `servo_states.voltage` is always 0; do not use it as a battery/health signal.

## Required before hardware use

- Resolve the quiet IMU topic or explicitly exclude the IMU from the design.
- Confirm whether `joint_states` is command echo (it is, from the vendor code)
  and measure the real feedback rate/latency with an independent read.
- Run the D-10 position-read semantics test (torque off, mechanically fixed,
  hand-turn) before trusting any measured joint value.
- Keep exactly one owner of `/dev/rrc`.
