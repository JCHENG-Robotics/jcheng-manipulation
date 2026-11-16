#!/usr/bin/env python3
"""Phase 0 acceptance check: plan a constrained 6D goal with MoveIt 2.

The target pose is produced by MoveIt's own ``/compute_fk`` service from the
requested joint configuration. That makes it exactly representable by the
model, so a failure points at the planning/IK configuration rather than at an
unreachable target.

The goal constrains position *and* orientation. This check is only meaningful
with ``position_only_ik: false`` (see config/kinematics.yaml); with
position-only IK it would pass while ignoring the tool orientation, which is
exactly what MASTER_PLAN_V3 §4 forbids for grasp execution.

This script does not execute the trajectory and does not touch hardware.

Usage:
    python3 tools/phase0/constrained_plan_check.py
    python3 tools/phase0/constrained_plan_check.py \
        --joints 0.0 0.6 -1.2 -0.6 0.5 --link gripper_link
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
IK_ERROR_MESSAGES = {
    1: "SUCCESS",
    0: "UNDEFINED",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -26: "START_STATE_INVALID",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}


def error_name(code):
    return IK_ERROR_MESSAGES.get(code, f"UNKNOWN({code})")


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_about_x(pose, degrees):
    """Rotate the target orientation about the base x axis (diagnostic control)."""
    half = math.radians(degrees) / 2.0
    q_rot = (math.sin(half), 0.0, 0.0, math.cos(half))
    q = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    x, y, z, w = quat_multiply(q_rot, q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    pose.orientation.x = x / norm
    pose.orientation.y = y / norm
    pose.orientation.z = z / norm
    pose.orientation.w = w / norm
    return pose


class ConstrainedPlanCheck(Node):
    def __init__(self, args):
        super().__init__("phase0_constrained_plan_check")
        self.args = args
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

    def wait_for_interfaces(self):
        self.get_logger().info("waiting for /compute_fk ...")
        if not self.fk_client.wait_for_service(timeout_sec=self.args.timeout):
            raise RuntimeError("/compute_fk not available")
        self.get_logger().info("waiting for /move_action ...")
        if not self.move_client.wait_for_server(timeout_sec=self.args.timeout):
            raise RuntimeError("/move_action not available")

    def compute_fk(self):
        request = GetPositionFK.Request()
        request.header.frame_id = self.args.frame
        request.fk_link_names = [self.args.link]
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = ARM_JOINTS + ["r_joint"]
        request.robot_state.joint_state.position = list(self.args.joints) + [0.0]

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("/compute_fk call did not complete")
        response = future.result()
        if response.error_code.val != 1:
            raise RuntimeError(
                f"/compute_fk failed: {error_name(response.error_code.val)}"
            )
        if not response.pose_stamped:
            raise RuntimeError("/compute_fk returned no pose")
        return response.pose_stamped[0].pose

    def plan(self, pose):
        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = not self.args.execute
        request = goal.request
        request.group_name = self.args.group
        request.pipeline_id = self.args.pipeline
        request.planner_id = self.args.planner
        request.num_planning_attempts = self.args.attempts
        request.allowed_planning_time = float(self.args.timeout)
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        position = PositionConstraint()
        position.header.frame_id = self.args.frame
        position.link_name = self.args.link
        position.target_point_offset.x = 0.0
        position.target_point_offset.y = 0.0
        position.target_point_offset.z = 0.0
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.args.position_tolerance]
        region.primitives.append(sphere)
        region.primitive_poses.append(pose)
        position.constraint_region = region
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.args.frame
        orientation.link_name = self.args.link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = self.args.orientation_tolerance
        orientation.absolute_y_axis_tolerance = self.args.orientation_tolerance
        orientation.absolute_z_axis_tolerance = self.args.orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        if self.args.constraints in ("both", "position"):
            constraints.position_constraints.append(position)
        if self.args.constraints in ("both", "orientation"):
            constraints.orientation_constraints.append(orientation)
        request.goal_constraints.append(constraints)

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=self.args.timeout)
        if not send_future.done() or send_future.result() is None:
            raise RuntimeError("failed to send MoveGroup goal")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("MoveGroup goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.args.timeout + 5.0)
        if not result_future.done() or result_future.result() is None:
            raise RuntimeError("MoveGroup result did not arrive")
        return result_future.result().result


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--joints",
        type=float,
        nargs=5,
        default=[0.0, 0.6, -1.2, -0.6, 0.5],
        metavar=("J1", "J2", "J3", "J4", "J5"),
        help="joint configuration used to generate the target pose via FK",
    )
    parser.add_argument("--group", default="arm")
    parser.add_argument("--pipeline", default="ompl")
    parser.add_argument("--planner", default="")
    parser.add_argument(
        "--constraints",
        choices=("both", "position", "orientation"),
        default="both",
        help="which constraints to send; diagnostic aid for 5-DoF limits",
    )
    parser.add_argument("--link", default="gripper_link")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--attempts", type=int, default=10)
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance", type=float, default=0.02)
    parser.add_argument("--json-out", default=None)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="execute the trajectory (mock hardware only; default is plan-only)",
    )
    parser.add_argument(
        "--rotate-x-deg",
        type=float,
        default=0.0,
        help="rotate the target orientation about base x; negative control for 5-DoF",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=[])
    node = ConstrainedPlanCheck(args)
    summary = {"joints": list(args.joints), "link": args.link, "frame": args.frame}
    try:
        node.wait_for_interfaces()
        pose = node.compute_fk()
        if args.rotate_x_deg:
            pose = rotate_about_x(pose, args.rotate_x_deg)
        summary["rotate_x_deg"] = args.rotate_x_deg
        summary["target_pose"] = {
            "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
            "orientation": {
                "x": pose.orientation.x,
                "y": pose.orientation.y,
                "z": pose.orientation.z,
                "w": pose.orientation.w,
            },
        }
        node.get_logger().info(
            "FK target: x=%.4f y=%.4f z=%.4f q=(%.4f, %.4f, %.4f, %.4f)"
            % (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        )
        started = time.monotonic()
        result = node.plan(pose)
        elapsed = time.monotonic() - started
        code = result.error_code.val
        points = len(result.planned_trajectory.joint_trajectory.points)
        summary.update(
            {
                "error_code": code,
                "error_name": error_name(code),
                "planning_time": result.planning_time,
                "wall_time": elapsed,
                "trajectory_points": points,
                "success": code == 1 and points > 0,
            }
        )
        print(
            "CONSTRAINED_PLAN "
            f"error_code={code} ({error_name(code)}) "
            f"planning_time={result.planning_time:.4f}s "
            f"trajectory_points={points}"
        )
        if args.json_out:
            with open(args.json_out, "w", encoding="utf-8") as handle:
                json.dump(summary, handle, indent=2)
        return 0 if summary["success"] else 1
    except Exception as exc:  # noqa: BLE001 - CLI check should report, not traceback
        summary["success"] = False
        summary["exception"] = str(exc)
        print(f"CONSTRAINED_PLAN FAILED: {exc}", file=sys.stderr)
        if args.json_out:
            with open(args.json_out, "w", encoding="utf-8") as handle:
                json.dump(summary, handle, indent=2)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
