# TF Tree — [U] measured; [V] model-only

> Model check: `xacro` + `check_urdf` succeed on the merged model (2026-11-15).
> No real TF was measured.

## Expected model tree

```text
world (MoveIt virtual joint, fixed)
└── base_link
    └── link1
        └── link2
            └── link3
                └── link4
                    ├── camera_connet_link      (vendor mount)
                    │   └── camera_link
                    │       └── camera_optical_frame   (REP-103)
                    └── link5
                        ├── end_effector_link
                        └── gripper_link
                            └── r_link / l_link / ...
```

## Checks

| Check | Result |
|---|---|
| Single root `base_link` | `[V]` `check_urdf` |
| No cycles | `[V]` `check_urdf` |
| `camera_link` / `camera_optical_frame` exist | `[V]` merged model |
| `base_link` → `camera_optical_frame` transform measured | `[U]` |
| `world` → `base_link` real transform | `[U]` (MoveIt uses 0,0,0 placeholder) |
| Joint names match real robot | `[U]` |

## Commands (after hardware is available)

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo base_link link5
```
