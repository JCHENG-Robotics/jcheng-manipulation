#!/usr/bin/env python3
"""S17: send one MoveGroup goal with planning_options.plan_only=true.

The arm is not moved: move_group runs with allow_trajectory_execution=false,
the controller action has no server, and the goal requests planning only.
"""
from __future__ import annotations

import json
import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
START = [0.0, 0.7309, -1.6127, -1.6591, 0.0]
TARGET = [0.10, 0.7309, -1.6127, -1.6591, 0.0]
OUT = "/home/ubuntu/s17-plan-only.json"


def error_name(value: int) -> str:
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == value:
            return name
    return f"UNKNOWN_{value}"


def main() -> int:
    node = Node("s17_plan_only_client")
    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=10.0):
        print("FAIL: /move_action server not available", file=sys.stderr)
        return 2

    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = "arm"
    request.num_planning_attempts = 3
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = 0.5
    request.max_acceleration_scaling_factor = 0.5
    request.start_state.is_diff = False
    request.start_state.joint_state.name = list(ARM_JOINTS)
    request.start_state.joint_state.position = list(START)

    constraints = Constraints()
    for name, position in zip(ARM_JOINTS, TARGET):
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = position
        joint.tolerance_above = 0.01
        joint.tolerance_below = 0.01
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    request.goal_constraints = [constraints]

    goal.planning_options.plan_only = True
    goal.planning_options.look_around = False
    goal.planning_options.replan = False
    goal.planning_options.replan_attempts = 0

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        print("FAIL: goal rejected", file=sys.stderr)
        return 3

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
    wrapped = result_future.result()
    if wrapped is None:
        print("FAIL: no result", file=sys.stderr)
        return 4

    result = wrapped.result
    trajectory = result.planned_trajectory.joint_trajectory
    points = len(trajectory.points)
    report = {
        "scenario": "S17",
        "action": "/move_action",
        "group_name": request.group_name,
        "plan_only_requested": bool(goal.planning_options.plan_only),
        "allow_trajectory_execution": False,
        "goal_start": dict(zip(ARM_JOINTS, START)),
        "goal_target": dict(zip(ARM_JOINTS, TARGET)),
        "error_code": int(result.error_code.val),
        "error_name": error_name(int(result.error_code.val)),
        "planning_time_s": float(result.planning_time),
        "trajectory_points": points,
        "trajectory_joint_names": list(trajectory.joint_names),
        "trajectory_start": {
            "joint_names": list(result.trajectory_start.joint_state.name),
            "positions": [float(v) for v in result.trajectory_start.joint_state.position],
        },
        "executed": False,
        "execution_note": "plan_only=true; allow_trajectory_execution=false; 0 controller servers",
    }
    with open(OUT, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True))

    if result.error_code.val != MoveItErrorCodes.SUCCESS:
        return 5
    if points <= 0:
        return 6
    return 0


if __name__ == "__main__":
    rclpy.init()
    try:
        sys.exit(main())
    finally:
        rclpy.shutdown()
