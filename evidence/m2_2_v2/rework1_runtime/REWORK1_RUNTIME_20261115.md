# M2.2 rework1 — Gazebo runtime regression (2026-11-15)

Workspace: `/home/jerichocheng/projects/ros2-industrial-arm-learning/ros2_ws`
Launch: `ros2 launch arm_description gazebo.launch.py controller_profile:=<profile>`
Goal: `/arm_controller/follow_joint_trajectory`, `joint1=0.2`, `joint2=0.2`, 2 s
Environment: ROS 2 Jazzy, Gazebo Sim 8.15.0, `gz_ros2_control/GazeboSimSystem`
Host: R9000P

## Results

| Profile | Controller config SHA-256 | Controllers active | Trajectory result |
|---|---|---|---|
| `default` | `c85bf8…` | 2/2 | SUCCEEDED, `error_code=0`, "Goal successfully reached!" |
| `gain_0p1` | `708408…` | 2/2 | SUCCEEDED, `error_code=0`, "Goal successfully reached!" |
| `gain_0p2` | `5ceea2…` | 2/2 | SUCCEEDED, `error_code=0`, "Goal successfully reached!" |

The launch log confirms the selected profile, its real path and SHA-256, and the
controller_manager switch sequence.

## Notes / known issues

1. A stale ArmPi MoveIt mock demo (`armpi_ultra_moveit_config demo.launch.py`)
   was running for ~21h and publishing `/robot_description` with
   `mock_components/GenericSystem`. It was stopped before these runs; otherwise
   the simple_arm Gazebo controller manager receives the wrong description.
2. On SIGINT teardown the launch logs
   `Gate 2 launch failure: stage=clock_bridge; returncode=0` because the
   registered `OnProcessExit` handlers shut down on both zero and non-zero exit.
   This is a teardown-reporting defect, not a trajectory failure. It should be
   fixed in a follow-up (only fail on non-zero).
3. The `clock_bridge` still uses `ignition.msgs.Clock` (JER-39 P3); it should be
   `gz.msgs.Clock` in Jazzy.
4. These are Gazebo/simulation results only. No hardware was connected.

## Raw logs

- `m2_2_default.log`, `m2_2_default_goal.log`
- `m2_2_gain_0p1.log`, `m2_2_gain_0p1_goal.log`
- `m2_2_traj.log`, `m2_2_goal.log` (gain_0p2)
