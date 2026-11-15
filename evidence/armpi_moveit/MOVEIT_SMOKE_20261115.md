# ArmPi MoveIt smoke test — 2026-11-15

Model: merged `armpi_ultra_description` (camera_link + camera_optical_frame,
joint3 upper=0.0, r_joint upper=1.257).
Launch: `ros2 launch armpi_ultra_moveit_config demo.launch.py
ros2_control_hardware_type:=mock_components use_sim_time:=false
use_gazebo:=false use_real:=false`

## Results

| Check | Result |
|---|---|
| `xacro` expand | success; 17 links, 16 joints |
| `check_urdf` | success; single root `base_link` |
| camera frames | `camera_connet_link` → `camera_link` → `camera_optical_frame` present |
| kinematics plugin | KDL loaded (`kdl_kinematics_plugin/KDLKinematicsPlugin`); no TRAC-IK load error |
| controllers | `arm_controller`, `gripper_controller`, `joint_state_broadcaster` loaded |
| joint-space plan | `/move_action` `MoveGroup` returned `error_code.val=1`, planning_time 0.0328 s |
| gripper accel limit error | not observed (previously `l_in_joint` failure) |

## Known issues still open

1. `sensors_3d.yaml` references `occupancy_map_monitor/DepthImageOctomapUpdater`;
   the plugin is not installed, so MoveIt logs an octomap updater error. No camera
   is connected; this does not block planning.
2. KDL warns that `base_link` has inertia at the root. This is a KDL limitation,
   not a planning failure. A dummy root link can be added later.
3. A **position-constrained** (OrientationConstraint) grasp target was not yet
   planned; only a joint-space goal was verified. `position_only_ik: true` is
   still a reachability-only setting.
4. No hardware was connected. These are mock_components results.

## Raw logs

- `armpi_plan2.log` — move_group/controller launch log
- `armpi_plan2_result.log` — action result (`error_code.val=1`)
- `armpi_merged.urdf` — expanded model
- `check_urdf.txt`
