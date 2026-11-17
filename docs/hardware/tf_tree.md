# TF Tree — live and model

> Status: the arm + camera TF chain was **verified live on 2026-11-17**. The
> camera **extrinsic accuracy is not measured**; the project
> `camera_optical_frame` is model-only and does not exist on the robot.

## Live tree (2026-11-17)

```text
base_link
└── link1
    └── link2
        └── link3
            └── link4
                ├── camera_connet_link          (vendor URDF mount)
                │   └── depth_camera_link       (driver static: x=0.038 m, pitch=+90°)
                │       └── rgb_camera_link     (identity)
                │           ├── depth_cam_color_frame      (identity)
                │           │   └── depth_cam_color_optical_frame (identity)
                │           └── depth_cam_depth_optical_frame      (identity)
                └── link5
                    ├── end_effector_link
                    └── gripper_link
                        └── r_link / l_link / ...
```

Frame facts:

- RGB `camera_info.header.frame_id` = **`rgb_camera_link`**.
- Depth `camera_info.header.frame_id` = **`depth_camera_link`**.
- The driver publishes `camera_connet_link → depth_camera_link` as a static
  transform. All remaining camera links in the driver chain are identity.
- **`camera_optical_frame` does not exist** on the live robot. The project
  model frames `camera_link` / `camera_optical_frame` are not published by the
  robot's `robot_state_publisher`.
- The camera is mounted on `link4`, before `joint5`; the camera transform is
  therefore pose-dependent (joint1–joint4).

## Live transform snapshots

At the reset pose (joint1–5 = 0, 2026-11-17 ~09:50):

| Target frame | Translation (m) | RPY (deg) |
|---|---|---|
| `rgb_camera_link` | `[0.029, -0.000, 0.370]` | `[0.156, 1.421, -0.449]` |
| `depth_camera_link` | `[0.039, -0.000, 0.369]` | `[-0.000, 1.376, -0.000]` |
| `depth_cam_color_optical_frame` | `[0.029, -0.000, 0.370]` | `[0.156, 1.421, -0.449]` |

At the boot pose (joint1 ≈ +0.8378, 2026-11-17 ~09:47):

| Target frame | Translation (m) | RPY (deg) |
|---|---|---|
| `depth_cam_color_optical_frame` | `[0.047, -0.021, 0.369]` | `[-0.132, 1.079, 47.996]` |

The yaw change (48° → ~0°) tracks joint1. The ~10 mm x difference between the
reset-pose RGB and depth snapshots was captured on separate `tf2_echo` calls
while the arm was settling; it is not a calibrated camera offset. Use the
static TF chain and a synchronized capture for any metric work.

## Model tree (project, nominal)

```text
world (MoveIt virtual joint, fixed)
└── base_link
    └── link1
        └── link2
            └── link3
                └── link4
                    ├── camera_connet_link
                    │   └── camera_link
                    │       └── camera_optical_frame   (REP-103, model-only)
                    └── link5
                        ├── end_effector_link
                        └── gripper_link
                            └── r_link / l_link / ...
```

## Checks

| Check | Result |
|---|---|
| Single root `base_link` | `[V]` live TF + `check_urdf` |
| No cycles | `[V]` `check_urdf` |
| `base_link → link5` | `[V]` live `tf2_echo`; example at boot pose `[0.074, 0.008, 0.338] m`, RPY `[-0.307, -0.101, 47.999]°` |
| `base_link → rgb_camera_link` | `[V]` live `tf2_echo` |
| `base_link → depth_cam_color_optical_frame` | `[V]` live `tf2_echo` |
| `camera_connet_link → depth_camera_link` | `[V]` driver static transform: `x=0.038 m`, `pitch=+1.5708 rad` |
| `base_link → camera_optical_frame` | `[V]` **does not exist** (negative control) |
| `world → base_link` real transform | `[U]` (MoveIt uses a fixed 0,0,0 virtual joint) |
| Joint names match the real robot | `[V]` `/joint_states` |
| Camera extrinsic accuracy | `[U]` (static config, not measured) |

## Commands

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link rgb_camera_link
ros2 run tf2_ros tf2_echo base_link depth_camera_link
ros2 run tf2_ros tf2_echo base_link depth_cam_color_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_optical_frame   # expected to fail
```

For RGB fiducial PnP use the `rgb_camera_link` frame (matching the RGB
`camera_info`). The `depth_cam_color_optical_frame` is identity to it. Do not
use `camera_optical_frame` on the real robot.
