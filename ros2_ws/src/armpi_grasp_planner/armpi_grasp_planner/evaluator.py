"""
MoveIt 2 evaluation pipeline for ArmPi grasp candidates.

The pipeline is deliberately staged, matching the master-plan order:

    geometry sanity
    -> full-pose IK
    -> FK verification of the IK solution
    -> collision-state validity
    -> plan-only trajectory generation (Pilz PTP by default)

Only candidates that reach the planning stage are sent to MoveIt's planner.
Nothing is executed: ``planning_options.plan_only`` is always true.
"""

import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from .candidates import assess_sanity
from .geometry import matrix_from_quaternion, rotation_angle_deg

ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
GRIPPER_JOINT = 'r_joint'
GRIPPER_OPEN_POSITION = 1.2

ERROR_NAMES = {
    1: 'SUCCESS',
    0: 'UNDEFINED',
    99999: 'FAILURE',
    -1: 'PLANNING_FAILED',
    -2: 'INVALID_MOTION_PLAN',
    -3: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
    -4: 'CONTROL_FAILED',
    -5: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
    -6: 'TIMED_OUT',
    -7: 'PREEMPTED',
    -10: 'START_STATE_IN_COLLISION',
    -11: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
    -12: 'GOAL_IN_COLLISION',
    -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -14: 'GOAL_CONSTRAINTS_VIOLATED',
    -15: 'INVALID_GROUP_NAME',
    -16: 'INVALID_GOAL_CONSTRAINTS',
    -17: 'INVALID_ROBOT_STATE',
    -18: 'INVALID_LINK_NAME',
    -19: 'INVALID_OBJECT_NAME',
    -21: 'FRAME_TRANSFORM_FAILURE',
    -22: 'COLLISION_CHECKING_UNAVAILABLE',
    -23: 'ROBOT_STATE_STALE',
    -24: 'SENSOR_INFO_STALE',
    -25: 'COMMUNICATION_FAILURE',
    -26: 'START_STATE_INVALID',
    -27: 'GOAL_STATE_INVALID',
    -28: 'UNRECOGNIZED_GOAL_TYPE',
    -29: 'CRASH',
    -30: 'ABORT',
    -31: 'NO_IK_SOLUTION',
}


def error_name(code):
    """Return a human-readable MoveIt error code."""
    return ERROR_NAMES.get(code, 'UNKNOWN({})'.format(code))


def _distance(a, b):
    return (
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    ) ** 0.5


def _pose(position, orientation):
    message = Pose()
    message.position.x, message.position.y, message.position.z = position
    (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
    ) = orientation
    return message


class MoveItGraspEvaluator(Node):
    """Synchronous MoveIt 2 client that screens grasp candidates."""

    def __init__(
        self,
        service_timeout=10.0,
        group='arm',
        tool_link='gripper_link',
        tcp_link='end_effector_link',
        frame='base_link',
    ):
        super().__init__('armpi_grasp_planner')
        self.service_timeout = service_timeout
        self.group = group
        self.tool_link = tool_link
        self.tcp_link = tcp_link
        self.frame = frame
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self._fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self._validity_client = self.create_client(
            GetStateValidity, '/check_state_validity'
        )
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self.latest_joint_state = None
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )

    def _joint_state_callback(self, message):
        state = JointState()
        state.name = list(message.name)
        state.position = list(message.position)
        state.velocity = list(message.velocity)
        state.effort = list(message.effort)
        self.latest_joint_state = state

    def wait_for_interfaces(self):
        """Block until the required MoveIt services and action exist."""
        interfaces = (
            ('/compute_ik', self._ik_client),
            ('/compute_fk', self._fk_client),
            ('/check_state_validity', self._validity_client),
        )
        for name, client in interfaces:
            if not client.wait_for_service(timeout_sec=self.service_timeout):
                raise RuntimeError('{} not available'.format(name))
        if not self._move_client.wait_for_server(
            timeout_sec=self.service_timeout
        ):
            raise RuntimeError('/move_action not available')
        deadline = time.monotonic() + self.service_timeout
        while self.latest_joint_state is None:
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def _call(self, client, request, timeout):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('service call did not complete')
        return future.result()

    def _robot_state(self, joints, gripper_position=GRIPPER_OPEN_POSITION):
        state = RobotState()
        state.joint_state.name = list(ARM_JOINTS) + [GRIPPER_JOINT]
        state.joint_state.position = [float(value) for value in joints] + [
            float(gripper_position)
        ]
        return state

    def _arm_joints(self, joint_state):
        index = {name: position
                 for position, name in enumerate(joint_state.name)}
        try:
            return [
                float(joint_state.position[index[name]])
                for name in ARM_JOINTS
            ]
        except KeyError as exc:
            raise RuntimeError(
                'IK solution is missing joint {}'.format(exc)
            ) from exc

    def compute_fk(self, joints, links=None):
        """Return ``{link: Pose}`` for the requested links."""
        links = list(links or (self.tool_link, self.tcp_link))
        request = GetPositionFK.Request()
        request.header.frame_id = self.frame
        request.fk_link_names = links
        request.robot_state = self._robot_state(joints)
        response = self._call(
            self._fk_client, request, self.service_timeout
        )
        if response.error_code.val != 1 or len(response.pose_stamped) != len(
            links
        ):
            raise RuntimeError(
                '/compute_fk failed: {}'.format(
                    error_name(response.error_code.val)
                )
            )
        return {
            link: response.pose_stamped[index].pose
            for index, link in enumerate(links)
        }

    def compute_ik(self, candidate_pose, seed, timeout):
        """Solve full-pose IK for one seed."""
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self.group
        ik_request.ik_link_name = self.tool_link
        ik_request.avoid_collisions = False
        stamped = PoseStamped()
        stamped.header.frame_id = self.frame
        stamped.pose = candidate_pose
        ik_request.pose_stamped = stamped
        ik_request.robot_state = self._robot_state(seed)
        whole_seconds = int(timeout)
        ik_request.timeout.sec = whole_seconds
        ik_request.timeout.nanosec = int((timeout - whole_seconds) * 1e9)
        started = time.monotonic()
        response = self._call(
            self._ik_client, request, timeout + self.service_timeout
        )
        elapsed = time.monotonic() - started
        code = response.error_code.val
        joints = None
        if code == 1:
            joints = self._arm_joints(response.solution.joint_state)
        return {
            'ok': joints is not None,
            'code': code,
            'name': error_name(code),
            'joints': joints,
            'elapsed_s': elapsed,
        }

    def check_fk(self, candidate, joints, position_tolerance, angle_tolerance):
        """Verify that the IK solution reproduces the candidate tool pose."""
        actual = self.compute_fk(joints, [self.tool_link])[self.tool_link]
        actual_position = (
            actual.position.x,
            actual.position.y,
            actual.position.z,
        )
        position_error = _distance(
            actual_position, candidate.gripper_position
        )
        target_rotation = matrix_from_quaternion(candidate.orientation)
        actual_rotation = matrix_from_quaternion(
            (
                actual.orientation.x,
                actual.orientation.y,
                actual.orientation.z,
                actual.orientation.w,
            )
        )
        orientation_error = rotation_angle_deg(target_rotation, actual_rotation)
        return {
            'ok': position_error <= position_tolerance
            and orientation_error <= angle_tolerance,
            'position_error_m': position_error,
            'orientation_error_deg': orientation_error,
            'position_tolerance_m': position_tolerance,
            'orientation_tolerance_deg': angle_tolerance,
        }

    def check_collision(self, joints):
        """Check self/environment collision validity of an IK solution."""
        request = GetStateValidity.Request()
        request.group_name = self.group
        request.robot_state = self._robot_state(joints)
        response = self._call(
            self._validity_client, request, self.service_timeout
        )
        contacts = [
            {
                'body_1': contact.contact_body_1,
                'body_2': contact.contact_body_2,
            }
            for contact in response.contacts
        ]
        return {
            'ok': bool(response.valid),
            'contact_count': len(contacts),
            'contacts': contacts,
        }

    def plan(
        self,
        candidate,
        pipeline,
        planner,
        allowed_planning_time,
        position_tolerance,
        orientation_tolerance,
    ):
        """Plan, but do not execute, a constrained trajectory to a candidate."""
        goal = MoveGroup.Goal()
        goal.planning_options.plan_only = True
        request = goal.request
        request.group_name = self.group
        if self.latest_joint_state is not None:
            request.start_state.joint_state = self.latest_joint_state
        request.pipeline_id = pipeline
        request.planner_id = planner
        request.num_planning_attempts = 1
        request.allowed_planning_time = allowed_planning_time
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        pose = _pose(candidate.gripper_position, candidate.orientation)
        position = PositionConstraint()
        position.header.frame_id = self.frame
        position.link_name = self.tool_link
        position.weight = 1.0
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [position_tolerance]
        region.primitives.append(sphere)
        region.primitive_poses.append(pose)
        position.constraint_region = region

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.frame
        orientation.link_name = self.tool_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        request.goal_constraints.append(constraints)

        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=self.service_timeout
        )
        if not send_future.done() or send_future.result() is None:
            return {
                'ok': False,
                'error_code': None,
                'error_name': 'GOAL_NOT_SENT',
                'trajectory_points': 0,
                'planning_time_s': 0.0,
            }
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return {
                'ok': False,
                'error_code': None,
                'error_name': 'REJECTED',
                'trajectory_points': 0,
                'planning_time_s': 0.0,
            }
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=allowed_planning_time + self.service_timeout,
        )
        if not result_future.done() or result_future.result() is None:
            return {
                'ok': False,
                'error_code': None,
                'error_name': 'RESULT_TIMEOUT',
                'trajectory_points': 0,
                'planning_time_s': 0.0,
            }
        result = result_future.result().result
        code = result.error_code.val
        points = len(result.planned_trajectory.joint_trajectory.points)
        return {
            'ok': code == 1 and points > 0,
            'error_code': code,
            'error_name': error_name(code),
            'trajectory_points': points,
            'planning_time_s': result.planning_time,
        }

    def evaluate_all(
        self,
        candidates,
        seeds,
        max_plan=5,
        max_pitch_deg=75.0,
        object_width_m=None,
        gripper_max_opening_m=None,
        pipeline='pilz_industrial_motion_planner',
        planner='PTP',
        allowed_planning_time=10.0,
        position_tolerance=0.001,
        orientation_tolerance=0.01,
        fk_position_tolerance=0.001,
        fk_orientation_tolerance_deg=1.0,
        ik_timeout=0.2,
    ):
        """
        Screen every candidate and return ``(results, counts)``.

        Each result contains only plan-only data. The robot is never moved.
        """
        results = []
        counts = {
            'generated': len(candidates),
            'sanity_ok': 0,
            'ik_ok': 0,
            'fk_ok': 0,
            'collision_ok': 0,
            'plan_attempted': 0,
            'plan_ok': 0,
            'duplicates_skipped': 0,
            'evaluator_errors': 0,
        }
        solved_states = set()
        if not seeds:
            raise ValueError('at least one IK seed is required')

        for candidate in candidates:
            entry = candidate.as_dict()
            entry['sanity'] = assess_sanity(
                candidate,
                max_pitch_deg=max_pitch_deg,
                object_width_m=object_width_m,
                gripper_max_opening_m=gripper_max_opening_m,
            )
            if not entry['sanity']['ok']:
                entry['status'] = 'SANITY_FAIL'
                results.append(entry)
                continue
            counts['sanity_ok'] += 1

            try:
                attempts = []
                ik_result = None
                candidate_seeds = []
                if candidate.seed_joints:
                    candidate_seeds.append(tuple(candidate.seed_joints))
                candidate_seeds.extend(tuple(seed) for seed in seeds)
                for index, seed in enumerate(candidate_seeds):
                    attempt = self.compute_ik(
                        _pose(candidate.gripper_position, candidate.orientation),
                        seed,
                        ik_timeout,
                    )
                    attempt['seed_index'] = index
                    attempt['seed_kind'] = (
                        'sampled'
                        if index == 0 and candidate.seed_joints
                        else 'default'
                    )
                    attempt['seed'] = [float(value) for value in seed]
                    attempts.append(attempt)
                    if attempt['ok']:
                        ik_result = attempt
                        break
                entry['ik_attempts'] = attempts
                entry['ik'] = ik_result or {
                    'ok': False,
                    'joints': None,
                    'code': attempts[-1]['code'],
                    'name': attempts[-1]['name'],
                }
                if not ik_result:
                    entry['status'] = 'IK_FAIL'
                    results.append(entry)
                    continue
                counts['ik_ok'] += 1

                fk_check = self.check_fk(
                    candidate,
                    ik_result['joints'],
                    fk_position_tolerance,
                    fk_orientation_tolerance_deg,
                )
                entry['fk_check'] = fk_check
                if not fk_check['ok']:
                    entry['status'] = 'FK_MISMATCH'
                    results.append(entry)
                    continue
                counts['fk_ok'] += 1

                collision = self.check_collision(ik_result['joints'])
                entry['collision'] = collision
                if not collision['ok']:
                    entry['status'] = 'COLLISION'
                    results.append(entry)
                    continue
                counts['collision_ok'] += 1

                if max_plan <= 0 or counts['plan_attempted'] >= max_plan:
                    entry['status'] = 'COLLISION_FREE'
                    results.append(entry)
                    continue

                state_key = tuple(
                    round(value, 4) for value in ik_result['joints']
                )
                if state_key in solved_states:
                    entry['status'] = 'SKIPPED_DUPLICATE_SOLUTION'
                    counts['duplicates_skipped'] += 1
                    results.append(entry)
                    continue
                solved_states.add(state_key)

                entry['plan'] = self.plan(
                    candidate,
                    pipeline,
                    planner,
                    allowed_planning_time,
                    position_tolerance,
                    orientation_tolerance,
                )
                counts['plan_attempted'] += 1
                if entry['plan']['ok']:
                    counts['plan_ok'] += 1
                    entry['status'] = 'PLAN_OK'
                else:
                    entry['status'] = 'PLAN_FAIL'
            except Exception as exc:  # noqa: BLE001 - report per candidate
                entry['status'] = 'EVALUATOR_ERROR'
                entry['error'] = str(exc)
                counts['evaluator_errors'] += 1
            results.append(entry)
        return results, counts
