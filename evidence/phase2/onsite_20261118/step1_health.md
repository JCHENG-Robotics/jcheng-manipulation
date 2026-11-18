# Step 1 — post-reboot read-only health check

Date: 2026-11-18 09:10–09:11 (robot local time, UTC+8).

## Host

```text
up 3 min, 2 users, load average: 0.48, 0.40, 0.19
systemctl is-active start_node.service  -> active
ActiveState=active  SubState=running  NRestarts=0
docker ps -> ArmPiUltra Up 4 minutes
df -h / -> /dev/mmcblk0p2 52G 44G 6.6G 87% /
eth0 -> 10.42.0.218/24, fe80::9bc7:cfd:b325:e3f0/64
```

## ROS graph

```text
/armpi_debug_gateway
/controller_manager
/joint_state_publisher
/kinematics
/robot_state_publisher
/ros_robot_controller
/servo_manager

/gripper_controller/follow_joint_trajectory
/arm_controller/follow_joint_trajectory
```

- `/joint_states`: **19.998–20.001 Hz**, min/max interval 0.050 s.
- Pose: `joint1=+0.837758`, `joint2..joint5=0`, `r_joint=-0.837758`,
  `l_joint=+0.837758` (mimic joints mirrored).
- Gateway health after reboot: `ros_ready=true`, `battery_ready=true`,
  `joints_ready=true`, `camera_ready=false` (camera not started),
  `motion_enabled=false`.

## Notes

- `ros2 node list` / `ros2 action info` occasionally fail with the known
  Humble CLI error `empty node name returned by the RMW layer`; the graph and
  the action servers themselves were healthy.
- The container has no `controller_manager_msgs` or `ros2controlcli`; the
  vendor `/controller_manager` is a custom node, not `ros2_control`.
- SSH occasionally stalls on banner exchange under DDS load; the IPv6
  link-local address is a working fallback after accepting its (identical)
  host key.
