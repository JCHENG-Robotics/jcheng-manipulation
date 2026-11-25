# M6 S17 — real MoveIt plan-only, 2026-11-25

Status: **pass** for the S17 plan-only scope. No robot motion command was sent.

This run replaces the previous L1-only S17 evidence (goal construction and
rejection tests) with one real `move_group` planning request on the robot's
MoveIt installation. It does not claim trajectory execution, S7, S9, S14 or a
full G8 run.

## 1. Environment

| Item | Value |
|---|---|
| Host | `raspberrypi`, container `ArmPiUltra` |
| ROS | Humble |
| MoveIt | `ros-humble-moveit-ros-move-group` 2.5.10-1jammy.20261107.004435 |
| DDS | CycloneDDS, `ROS_DOMAIN_ID=42` (capability domain; no vendor bridge) |
| Config | `armpi_ultra_moveit_config` 0.3.0, vendor copy in `/home/ubuntu/jcheng-moveit` |
| Planner | OMPL `geometric::RRTConnect` (log line 79) |
| Launch | `move_group.launch.py allow_trajectory_execution:=false`; no controller manager |

The vendor config asks for `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`.
No TRAC-IK package exists for Ubuntu 22.04 arm64 in the ROS repository, so the
local config copy uses the shipped
`kdl_kinematics_plugin/KDLKinematicsPlugin` with the vendor's
`position_only_ik: true`. The original file is preserved next to it as
`kinematics.yaml.trac-ik`. This is a local planning-config adaptation, not a
change to `jcheng-manipulation` or the v1.0.0 baseline.

The robot's Wi-Fi is behind a captive portal. MoveIt was installed from
`packages.ros.org` and `ports.ubuntu.com` through a temporary HTTP proxy on the
laptop's direct link (`10.42.0.1:3128`); the original apt sources and DNS
configuration were restored after the install. This network detail does not
affect the S17 result.

## 2. Request

`moveit_plan_only.py` sends one `moveit_msgs/action/MoveGroup` goal to
`/move_action`:

- group `arm`
- start state: `joint1..joint5 = 0.0, 0.7309, -1.6127, -1.6591, 0.0`
- target: the same pose with `joint1 = 0.10` (0.01 rad tolerance)
- `planning_options.plan_only = true`
- `look_around = false`, `replan = false`
- 3 attempts, 5 s planning budget, 0.5 velocity/acceleration scaling

The start state is supplied in the request (`is_diff=false`), so the missing
`/joint_states` publisher does not gate the plan. MoveIt logged a warning that
it could not fetch a current robot state and then planned from the request's
start state.

## 3. Result

`error_code.val = 1` (`SUCCESS`); planning time `0.050016037 s`; the returned
joint trajectory has **8 points** over `joint1..joint5`. The complete response
is in [s17-plan-only.json](s17-plan-only.json).

Evidence that no execution path was available:

| Check | Result |
|---|---|
| `allow_trajectory_execution` parameter on `/move_group` | `False` |
| `/arm_controller/follow_joint_trajectory` | 1 client (`moveit_simple_controller_manager`), **0 servers** |
| `/gripper_controller/follow_joint_trajectory` | 1 client, **0 servers** |
| command topics matching `arm_controller`, `servo_controller`, `joint_trajectory` | none |
| request `plan_only` | `true` |
| controller manager | not started |

The MoveIt log line `MoveGroupMoveAction: executing..` is the action
capability receiving the request, not a trajectory being executed. The same
log ends with `Motion plan was computed successfully.` and contains no
execution or controller-send event. The no-server, no-topic checks are the
stronger evidence.

Commands and outputs are in [s17-preflight.txt](s17-preflight.txt); the raw
client output is [s17-plan-only-log.txt](s17-plan-only-log.txt); the full
move_group log is [move_group-log.txt](move_group-log.txt).

## 4. Reproduction

On the robot container, with the config built:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source /home/ubuntu/jcheng-moveit/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch armpi_ultra_moveit_config move_group.launch.py \
  allow_trajectory_execution:=false
```

Then run `moveit_plan_only.py` in the same environment. The launch and client
sources are included as [run_move_group.sh](run_move_group.sh) and
[moveit_plan_only.py](moveit_plan_only.py).

## 5. Limitations

- One joint goal only; no Cartesian, pose-goal or multi-waypoint plan.
- No planning-scene obstacles (no table or cube); collision checking is against
  the robot model only.
- KDL rather than the vendor's TRAC-IK because of the arm64 package gap.
- Plan-only does not exercise trajectory execution, controllers or the vendor
  bridge. S14, S7, S9 and the full G8 flow remain open.
- `allow_trajectory_execution=false` and the absence of controller servers are
  process-level guards; they are not a hardware safety certification.
