#!/usr/bin/env python3
"""Capture chessboard samples for camera/hand-eye calibration.

Run this inside the robot container (it has OpenCV, the vendor
FollowJointTrajectory action server and the bus-servo state service):

    sudo docker cp tools/phase2/chessboard_capture.py ArmPiUltra:/tmp/
    sudo docker exec -it -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; python3 /tmp/chessboard_capture.py \
         --output-dir /tmp/chessboard_calib_servo_20261117 \
         --base-pose 0.0,0.59888,-1.25664,-0.71209 \
         --delta 0.12 --pre-offset 0.08 --pre-joint1-only --final-base'

The script moves the arm through small joint offsets around a capture base
pose, or through absolute targets from ``--poses-file`` (the offline
``calibration_pose_selector.py`` output), waits for the arm and auto-exposure
to settle, captures three RGB frames per pose, detects the 11x8 chessboard
(20 mm squares) with IPPE + LM, and reads the physical bus-servo positions
through ``/ros_robot_controller/bus_servo/get_state``. The dataset stores both
the commanded joint_states and ``measured_joints``; the solver prefers the
measured values because the controller's joint_states are cached commands.
Raw images stay in the private output directory; do not commit them if they
contain people.
"""

import argparse
import json
import os
import time

import cv2
import numpy as np

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
# Vendor servo_controller.yaml: joint1..joint5 = IDs 6,5,4,3,2.
SERVO_IDS = {
    'joint1': 6,
    'joint2': 5,
    'joint3': 4,
    'joint4': 3,
    'joint5': 2,
}
SERVO_INIT_PULSE = 500
RADIANS_PER_TICK = 240.0 / 360.0 * (2.0 * 3.141592653589793) / 1000.0
BOARD_COLS = 11
BOARD_ROWS = 8
SQUARE_M = 0.020
ARM_LIMITS = {
    'joint1': (-2.09, 2.09),
    'joint2': (-2.09, 2.09),
    'joint3': (-2.09, 0.0),
    'joint4': (-2.09, 2.09),
    'joint5': (-2.09, 2.09),
}


def image_to_bgr(msg):
    """Convert a sensor_msgs/Image to an HxWx3 BGR array."""
    if msg.encoding == 'bgr8':
        row = msg.width * 3
        array = np.frombuffer(msg.data, dtype=np.uint8)
        array = array.reshape(msg.height, msg.step)[:, :row]
        return array.reshape(msg.height, msg.width, 3).copy()
    if msg.encoding == 'rgb8':
        row = msg.width * 3
        array = np.frombuffer(msg.data, dtype=np.uint8)
        array = array.reshape(msg.height, msg.step)[:, :row]
        return cv2.cvtColor(
            array.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    raise ValueError('unsupported image encoding {}'.format(msg.encoding))


def pulses_to_joints(pulses):
    """Convert raw bus-servo pulses to joint radians (vendor convention).

    ``pulses`` maps servo ID to the position read from the servo. Joint3 is
    mounted reversed in the vendor ``servo_controller.yaml`` (min 1000,
    max 500), so its sign is inverted.
    """
    joints = {}
    for name, servo_id in SERVO_IDS.items():
        pulse = pulses.get(servo_id)
        if pulse is None:
            continue
        if name == 'joint3':
            joints[name] = (SERVO_INIT_PULSE - pulse) * RADIANS_PER_TICK
        else:
            joints[name] = (pulse - SERVO_INIT_PULSE) * RADIANS_PER_TICK
    return joints


def median_pulses(readings):
    """Return the per-servo median of several pulse readings."""
    ids = set()
    for reading in readings:
        ids.update(reading)
    out = {}
    for servo_id in ids:
        values = [
            reading[servo_id] for reading in readings
            if servo_id in reading
        ]
        if values:
            out[servo_id] = int(np.median(values))
    return out


def detect_board(image, camera_matrix, dist_coeffs):
    """Detect the 11x8 chessboard and return its pose in the camera frame."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(
        gray, (BOARD_COLS, BOARD_ROWS), flags)
    if not found:
        return None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1), criteria)
    object_points = np.zeros((BOARD_COLS * BOARD_ROWS, 3), np.float32)
    object_points[:, :2] = np.mgrid[
        0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2) * SQUARE_M
    # IPPE is the planar-target solver and does not need an initial guess.
    # Using the previous frame as an extrinsic guess biased the first capture
    # after a move by several millimetres.
    solved, rvec, tvec = cv2.solvePnP(
        object_points, corners, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE)
    if not solved:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(
        object_points, corners, camera_matrix, dist_coeffs, rvec, tvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs)
    error = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))
    return {
        'rvec': [float(value) for value in rvec.reshape(3)],
        'tvec': [float(value) for value in tvec.reshape(3)],
        'rms_px': rms,
        'corners': [
            [float(value) for value in point]
            for point in corners.reshape(-1, 2)
        ],
    }


def combine_detections(detections):
    """Combine repeated board detections into one median sample."""
    valid = [board for board in detections if board is not None]
    if not valid:
        return None
    translations = np.asarray(
        [board['tvec'] for board in valid], dtype=np.float64)
    median = np.median(translations, axis=0)
    distances = np.linalg.norm(translations - median, axis=1)
    best = int(np.argmin(distances))
    out = dict(valid[best])
    out['tvec'] = [float(value) for value in median]
    out['repeats'] = len(valid)
    out['translation_spread_m'] = float(distances.max())
    out['rms_px_max'] = float(max(board['rms_px'] for board in valid))
    out['repeat_tvec'] = [
        [float(value) for value in board['tvec']] for board in valid
    ]
    return out


class ChessboardCapture(Node):
    """Move the arm, capture frames and detect the chessboard."""

    def __init__(self, output_dir, duration, settle=0.5, warmup=3,
                 repeats=3, servo_reads=3):
        """Create subscriptions and the arm action client."""
        super().__init__('chessboard_calib_capture')
        self.output_dir = output_dir
        self.duration = duration
        self.settle = settle
        self.warmup = warmup
        self.repeats = repeats
        self.servo_reads = servo_reads
        self.joint_state = None
        self.camera_info = None
        self.latest_image = None
        self.latest_image_wall = 0.0
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(
            CameraInfo, '/depth_cam/rgb/camera_info', self._info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self._image_cb,
            qos_profile_sensor_data)
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.servo_client = self.create_client(
            GetBusServoState,
            '/ros_robot_controller/bus_servo/get_state')

    def _joint_cb(self, msg):
        self.joint_state = msg

    def _info_cb(self, msg):
        self.camera_info = msg

    def _image_cb(self, msg):
        self.latest_image = msg
        self.latest_image_wall = time.monotonic()

    def spin_until(self, predicate, timeout):
        """Poll ROS callbacks until predicate() is true or timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def wait_ready(self):
        """Wait for joint state, camera info and one image."""
        if not self.spin_until(
                lambda: self.joint_state is not None
                and self.camera_info is not None
                and self.latest_image is not None, 10.0):
            raise RuntimeError('joint state / camera info / image not received')

    def arm_positions(self):
        """Return the five arm joint positions."""
        if self.joint_state is None:
            raise RuntimeError('no joint state')
        index = {name: i for i, name in enumerate(self.joint_state.name)}
        return [float(self.joint_state.position[index[name]])
                for name in ARM_JOINTS]

    def wait_positions(self, target, tolerance=0.02, timeout=2.0):
        """Wait until the reported joints are within tolerance of target."""
        def close():
            return all(
                abs(value - expected) <= tolerance
                for value, expected in zip(self.arm_positions(), target))
        return self.spin_until(close, timeout)

    def read_servo_pulses(self):
        """Read the arm bus-servo positions from the controller service."""
        if not self.servo_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('bus servo state service not available')
        request = GetBusServoState.Request()
        for servo_id in sorted(SERVO_IDS.values()):
            cmd = GetBusServoCmd()
            cmd.id = int(servo_id)
            cmd.get_position = 1
            request.cmd.append(cmd)
        future = self.servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('bus servo state read failed')
        pulses = {}
        for cmd, state in zip(request.cmd, response.state):
            if state.position:
                pulses[int(cmd.id)] = int(state.position[0])
        if len(pulses) != len(request.cmd):
            raise RuntimeError(
                'partial bus servo read: {} of {}'.format(
                    len(pulses), len(request.cmd)))
        return pulses

    def sample_servo_pulses(self):
        """Read the servo positions several times and take the median."""
        readings = [
            self.read_servo_pulses() for _ in range(max(1, self.servo_reads))
        ]
        return median_pulses(readings)

    def move_to(self, target, duration=None):
        """Send one FollowJointTrajectory point and wait for completion."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('arm action server not available')
        duration = self.duration if duration is None else duration
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in target]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1.0) * 1e9))
        trajectory.points = [point]
        goal.trajectory = trajectory
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rejected')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 10.0)
        result = result_future.result()
        if result is None:
            raise RuntimeError('trajectory result timeout')
        code = getattr(result.result, 'error_code', 0)
        if code not in (0, FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError(
                'trajectory failed: {} {}'.format(
                    code, getattr(result.result, 'error_string', '')))

    def capture(self, index, target):
        """Capture a fresh frame and detect the board."""
        info = self.camera_info
        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(info.d, dtype=np.float64).reshape(-1)
        # Let the arm stop vibrating and the auto-exposure settle after a
        # move; the first frame after motion showed a ~2-4 mm pose shift.
        time.sleep(self.settle)
        for _ in range(self.warmup):
            start = time.monotonic()
            if not self.spin_until(
                    lambda: self.latest_image_wall > start, 2.0):
                raise RuntimeError('no fresh image during warmup')
        servo_start = self.sample_servo_pulses()
        detections = []
        images = []
        for _ in range(max(1, self.repeats)):
            start = time.monotonic()
            if not self.spin_until(
                    lambda: self.latest_image_wall > start, 2.0):
                raise RuntimeError('no fresh image')
            image = image_to_bgr(self.latest_image)
            images.append(image)
            detections.append(
                detect_board(image, camera_matrix, dist_coeffs))
        for repeat, image in enumerate(images):
            image_name = 'sample_{:02d}_r{}.png'.format(index, repeat)
            cv2.imwrite(
                os.path.join(self.output_dir, image_name), image)
        image_name = 'sample_{:02d}_r0.png'.format(index)
        board = combine_detections(detections)
        servo_end = self.sample_servo_pulses()
        command_joints = self.arm_positions()
        start_joints = pulses_to_joints(servo_start)
        end_joints = pulses_to_joints(servo_end)
        measured_joints = [
            float(0.5 * (start_joints[name] + end_joints[name]))
            for name in ARM_JOINTS
        ]
        servo_shift = max(
            abs(servo_start[servo_id] - servo_end[servo_id])
            for servo_id in SERVO_IDS.values())
        sample = {
            'index': index,
            'target_joints': [float(value) for value in target],
            'joints': command_joints,
            'command_joints': command_joints,
            'servo_pulses_start': {
                str(key): value for key, value in servo_start.items()
            },
            'servo_pulses_end': {
                str(key): value for key, value in servo_end.items()
            },
            'measured_joints_start': [
                float(start_joints[name]) for name in ARM_JOINTS
            ],
            'measured_joints_end': [
                float(end_joints[name]) for name in ARM_JOINTS
            ],
            'measured_joints': measured_joints,
            'servo_shift_pulses': int(servo_shift),
            'joint_names': list(self.joint_state.name),
            'joint_positions': [
                float(value) for value in self.joint_state.position
            ],
            'camera_frame': info.header.frame_id,
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.tolist(),
            'board': board,
            'image': image_name,
            'image_stamp': {
                'sec': self.latest_image.header.stamp.sec,
                'nanosec': self.latest_image.header.stamp.nanosec,
            },
        }
        return sample


def pose_offsets(delta):
    """Joint-space offsets around the capture base pose.

    ``delta`` drives the yaw (joint1) and roll (joint4) amplitude; the pitch
    joints 2/3 use half of it. At the 2026-11-17 base pose this kept all
    corners inside a 25 px image margin (predicted from the live detection).
    """
    pitch = 0.5 * delta
    values = [
        (0.0, 0.0, 0.0, 0.0),
        (delta, 0.0, 0.0, 0.0),
        (-delta, 0.0, 0.0, 0.0),
        (0.0, pitch, 0.0, 0.0),
        (0.0, -pitch, 0.0, 0.0),
        (0.0, 0.0, pitch, 0.0),
        (0.0, 0.0, -pitch, 0.0),
        (0.0, 0.0, 0.0, delta),
        (0.0, 0.0, 0.0, -delta),
        (delta, -pitch, pitch, -delta),
        (-delta, pitch, -pitch, delta),
        (0.0, pitch, pitch, 0.0),
        (0.0, pitch, -pitch, 0.0),
        (0.0, -pitch, pitch, 0.0),
        (delta, pitch, -pitch, delta),
    ]
    return [list(offset) for offset in values]


def load_poses_file(path):
    """Load absolute five-joint targets from a pose-selection JSON.

    Accepts the full selector report (top-level ``poses``) or a bare list.
    Each entry must contain ``joints`` (or ``target_joints``).
    """
    with open(path, 'r', encoding='utf-8') as handle:
        data = json.load(handle)
    entries = data.get('poses') if isinstance(data, dict) else data
    if not isinstance(entries, list) or not entries:
        raise RuntimeError(
            '{} has no non-empty poses list'.format(path))
    poses = []
    for index, entry in enumerate(entries):
        if isinstance(entry, dict):
            joints = entry.get('joints', entry.get('target_joints'))
        else:
            joints = entry
        if joints is None or len(joints) != len(ARM_JOINTS):
            raise RuntimeError(
                'pose {} needs {} joint values'.format(
                    index, len(ARM_JOINTS)))
        values = [float(value) for value in joints]
        if not all(np.isfinite(values)):
            raise RuntimeError(
                'pose {} has non-finite joint values'.format(index))
        poses.append(values)
    return poses


def validate_target(target, label):
    """Raise when a target is outside the provisional joint limits."""
    for name, value in zip(ARM_JOINTS, target):
        lower, upper = ARM_LIMITS[name]
        if not lower <= value <= upper:
            raise RuntimeError(
                '{} {}={} outside [{}, {}]'.format(
                    label, name, value, lower, upper))


def main():
    """Run the capture sequence."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output-dir', default='/tmp/chessboard_calib_20261117')
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--settle', type=float, default=0.5)
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--servo-reads', type=int, default=3)
    parser.add_argument('--delta', type=float, default=0.08)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--final-base', action='store_true')
    parser.add_argument(
        '--check-only', action='store_true',
        help='capture stationary frames without moving the arm')
    parser.add_argument(
        '--count', type=int, default=2,
        help='number of stationary frames for --check-only')
    parser.add_argument(
        '--pre-offset', type=float, default=0.0,
        help='approach each target from target - offset (rad) on joints 1-4')
    parser.add_argument(
        '--pre-joint1-only', action='store_true',
        help='apply the pre-offset only to joint1')
    parser.add_argument(
        '--base-pose', default='',
        help='comma-separated joint1..joint5 capture base pose; '
             'the arm moves there before the sequence')
    parser.add_argument(
        '--poses-file', default='',
        help='absolute five-joint targets from '
             'calibration_pose_selector.py; incompatible with --base-pose '
             'and --check-only')
    args = parser.parse_args()
    if args.poses_file and args.base_pose:
        parser.error('--poses-file and --base-pose are mutually exclusive')
    if args.poses_file and args.check_only:
        parser.error('--poses-file and --check-only are mutually exclusive')

    os.makedirs(args.output_dir, exist_ok=True)
    rclpy.init()
    node = ChessboardCapture(
        args.output_dir, args.duration, args.settle, args.warmup,
        args.repeats, args.servo_reads)
    samples = []
    base = None
    try:
        node.wait_ready()
        base = node.arm_positions()
        print('START', [round(value, 5) for value in base])
        if args.poses_file:
            targets = load_poses_file(args.poses_file)
            if args.test:
                targets = targets[:2]
            # The first selected pose is the validated base; return there for
            # the held-out final-base capture instead of the arbitrary startup
            # pose.
            base = list(targets[0])
            print('POSES', len(targets), args.poses_file)
        else:
            if args.base_pose:
                base_target = [
                    float(value) for value in args.base_pose.split(',')
                ]
                if len(base_target) == 4:
                    base_target.append(float(base[4]))
                if len(base_target) != len(ARM_JOINTS):
                    raise RuntimeError(
                        '--base-pose needs 4 or 5 joint values')
                validate_target(base_target, 'base-pose')
                if args.pre_offset > 0.0:
                    pre_base = list(base_target)
                    if args.pre_joint1_only:
                        pre_base[0] -= args.pre_offset
                    else:
                        pre_base = [
                            base_target[i] - args.pre_offset
                            for i in range(4)
                        ] + [base_target[4]]
                    validate_target(pre_base, 'base pre-pose')
                    print('APPROACH BASE PRE', [round(value, 4)
                                                for value in pre_base])
                    node.move_to(
                        pre_base, duration=max(6.0, 0.5 * args.duration))
                print('APPROACH BASE', [round(value, 4)
                                        for value in base_target])
                node.move_to(base_target, duration=max(8.0, args.duration))
                node.wait_positions(base_target)
                base = node.arm_positions()
            print('BASE', [round(value, 5) for value in base])
            offsets = pose_offsets(args.delta)
            if args.test:
                offsets = offsets[:2]
            if args.check_only:
                offsets = []
            targets = [
                [base[i] + offset[i] for i in range(4)] + [base[4]]
                for offset in offsets
            ]
        for index, target in enumerate(targets):
            validate_target(target, 'target')
            if args.pre_offset > 0.0:
                pre = list(target)
                if args.pre_joint1_only:
                    pre[0] -= args.pre_offset
                else:
                    pre = [
                        target[i] - args.pre_offset for i in range(4)
                    ] + [target[4]]
                validate_target(pre, 'pre-pose')
                print('PRE', index, [round(value, 4) for value in pre])
                node.move_to(pre)
            current = node.arm_positions()
            unchanged = max(
                abs(target[i] - current[i]) for i in range(len(ARM_JOINTS))
            ) <= 0.005
            if unchanged and args.pre_offset <= 0.0:
                print('SAMPLE', index, 'at current pose')
            else:
                print('MOVE', index, [round(value, 4) for value in target])
                node.move_to(target)
            sample = node.capture(index, target)
            samples.append(sample)
            board = sample['board']
            if board is None:
                print('  board NOT detected')
            else:
                print(
                    '  board ok rms={:.3f}px spread={:.2f}mm shift={}p '
                    'tvec={}'.format(
                        board['rms_px'],
                        1000.0 * board.get('translation_spread_m', 0.0),
                        sample.get('servo_shift_pulses', -1),
                        [round(value, 4) for value in board['tvec']]))
        if args.check_only:
            for index in range(args.count):
                sample = node.capture(index, base)
                samples.append(sample)
                board = sample['board']
                if board is None:
                    print('CHECK {} board NOT detected'.format(index))
                else:
                    print(
                        'CHECK {} rms={:.3f}px spread={:.2f}mm shift={}p '
                        'tvec={}'.format(
                            index, board['rms_px'],
                            1000.0 * board.get(
                                'translation_spread_m', 0.0),
                            sample.get('servo_shift_pulses', -1),
                            [round(value, 4) for value in board['tvec']]))
    finally:
        if base is not None and not args.check_only:
            try:
                print('RETURN TO BASE')
                if args.pre_offset > 0.0:
                    pre_base = list(base)
                    if args.pre_joint1_only:
                        pre_base[0] -= args.pre_offset
                    else:
                        pre_base = [
                            base[i] - args.pre_offset for i in range(4)
                        ] + [base[4]]
                    node.move_to(pre_base)
                node.move_to(base)
                if args.final_base:
                    final = node.capture(len(samples), base)
                    final['final_base'] = True
                    samples.append(final)
                    board = final['board']
                    print(
                        'FINAL BASE board {}'.format(
                            'NOT detected' if board is None else
                            'rms={:.3f}px tvec={}'.format(
                                board['rms_px'],
                                [round(value, 4)
                                 for value in board['tvec']])))
            except Exception as exc:  # noqa: BLE001 - best-effort recovery
                print('return-to-base failed:', exc)
        with open(os.path.join(args.output_dir, 'samples.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump({
                'board_cols': BOARD_COLS,
                'board_rows': BOARD_ROWS,
                'square_m': SQUARE_M,
                'duration_s': args.duration,
                'delta_rad': args.delta,
                'pre_offset_rad': args.pre_offset,
                'pre_joint1_only': args.pre_joint1_only,
                'pose_source': (
                    'poses_file' if args.poses_file else 'joint_offsets'),
                'poses_file': os.path.abspath(args.poses_file)
                if args.poses_file else None,
                'samples': samples,
            }, handle, indent=2)
        node.destroy_node()
        rclpy.shutdown()
        print('WROTE', os.path.join(args.output_dir, 'samples.json'))


if __name__ == '__main__':
    main()
