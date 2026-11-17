# Real `camera_forward` named-state check — 2026-11-17

> Scope: real ArmPi Ultra, **plan-only**. `move_group` ran in the Humble Docker
> container against the live robot state and `/plan_kinematic_path` returned
> `SUCCESS`. No execute goal was sent, so the arm did not move. This is not a
> hardware motion validation.

## Environment

| Item | Value |
|---|---|
| Robot | `10.51.128.226` (WiFi `wlan0`), `ArmPiUltra` container, ROS 2 Humble |
| Laptop | `10.51.128.246` (`wlp4s0`), container `armpi-humble-moveit` |
| MoveIt launch | `real_moveit.launch.py use_rviz:=false` |
| SRDF | `armpi_ultra_moveit_config/config/armpi_ultra.srdf` |
| Service | `/plan_kinematic_path` (`moveit_msgs/srv/GetMotionPlan`) |
| Execution | none; plan-only |

## Command

```bash
sg docker -c "docker cp \
  evidence/armpi_moveit/real_camera_forward_20261117/plan_named_state_check.py \
  armpi-humble-check:/tmp/plan_named_state_check.py"
sg docker -c "docker exec armpi-humble-check bash -lc \
  'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && \
   python3 /tmp/plan_named_state_check.py camera_forward'"
```

## Result

| Item | Value |
|---|---|
| Named state | `camera_forward`, group `arm` |
| Target | `joint1=0`, `joint2=0.419`, `joint3=-1.257`, `joint4=-0.709`, `joint5=0` |
| `error_code` | `1` (`MoveItErrorCodes.SUCCESS`) |
| Planning time | 0.015 s |
| Waypoints | 96 |
| Final point | `joint1=-0.008899`, `joint2=0.419328`, `joint3=-1.259860`, `joint4=-0.701461`, `joint5=0.004995` |
| Start state | live `/joint_states`: `joint1=-0.858702`, `joint2=+0.448201`, `joint3=-1.528908`, `joint4=-0.557109`, `joint5=+1.118407` |

The final point is within the 0.01 rad joint tolerance. The camera pose at the
target was checked by FK: camera mount at x≈0.154 m, z≈0.282 m in `base_link`,
with the optical +z axis within 0.1° of horizontal. The camera is on `link4`,
so `joint5` does not change the view direction. The FK model was cross-checked
against the live `base_link → camera_connet_link` TF at the start pose
(`[0.134, -0.077, 0.256]` m), which matches.

`move_group` loaded both `arm_controller` and `gripper_controller`; the robot
reported exactly one MoveIt client on `/arm_controller/follow_joint_trajectory`
(`/moveit_simple_controller_manager`).

## Interpretation

- The named state is present in the semantic model loaded by
  `real_moveit.launch.py` and is reachable from the current real pose.
- It is **not** a small motion from the current pose: `joint1` changes by
  ≈0.86 rad and `joint5` by ≈1.12 rad. Execute it in RViz at 0.05 scaling with
  the workspace clear.
- Real execution through RViz is still unverified; the next step is a
  supervised **Plan → inspect → Execute** with the camera-forward target.

## Files

| File | Content |
|---|---|
| `plan_camera_forward_20261117.txt` | Command, target, service result and start state |
| `plan_named_state_check.py` | Repeatable plan-only check for any SRDF named state |
