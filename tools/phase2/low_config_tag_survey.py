#!/usr/bin/env python3
"""Survey the model error over the low (grasp) configuration with an AprilTag.

Run this inside the robot container (it has OpenCV, the vendor
FollowJointTrajectory action server and the bus-servo state service):

    sudo docker cp tools/phase2/low_config_tag_survey.py ArmPiUltra:/tmp/
    sudo docker exec -d -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; \
       exec python3 /tmp/low_config_tag_survey.py \
         --output-dir /tmp/low_tag_survey_20261119 \
         --poses-file /tmp/low_tag_poses.json \
         > /tmp/low_tag_survey.log 2>&1'

The tag is a **fixed** target on the table. The arm moves through poses that
share one tag placement: a reference pose inside the validated table-down
volume (where the model is accurate to 1-3 mm) and the grasp family (where the
model error is 8-15 mm). For each pose the script captures RGB frames, detects
the tag, and reads the physical bus-servo positions. The dataset stores the
measured joints so the host can compute the model prediction with the project
URDF.

The tag centre in ``base_link`` should be constant. Its spread across the
reference poses is the reference uncertainty; the per-pose offset from the
reference mean is the model error at that configuration. Raw images stay in
the private output directory; do not commit them.

Pose file format (absolute five-joint targets, joint5 is not moved)::

    {"poses": [{"name": "G_grasp",
                "joints": [0.0, -0.6618, -1.8012, -0.3917, 0.0]}, ...]}
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
# Adopted commandable range (EEPROM limit - 20 pulses, joint2 low measured);
# see ros2_ws/src/armpi_ultra_moveit_config/config/joint_limits.yaml.
ARM_LIMITS = {
    'joint1': (-1.8012, 1.8012),
    'joint2': (-0.6618, 1.2985),
    'joint3': (-1.8012, 0.0),
    'joint4': (-1.5917, 1.5917),
    'joint5': (-1.5917, 1.5917),
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
    """Convert raw bus-servo pulses to joint radians (vendor convention)."""
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


def make_detector():
    """Build an AprilTag 36h11 detector across OpenCV versions."""
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters())
    params = cv2.aruco.DetectorParameters_create()
    return dictionary, params


def detect_tag(image, camera_matrix, dist_coeffs, marker_id, size_m):
    """Detect one AprilTag and return its centre pose in the camera frame."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detector = make_detector()
    if isinstance(detector, tuple):
        dictionary, params = detector
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=params)
    else:
        corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or marker_id not in ids.ravel():
        return None
    index = int(np.where(ids.ravel() == marker_id)[0][0])
    points = corners[index][0].astype(np.float64)
    half = size_m / 2.0
    object_points = np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)
    solved, rvec, tvec = cv2.solvePnP(
        object_points, points, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE)
    if not solved:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(
        object_points, points, camera_matrix, dist_coeffs, rvec, tvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs)
    error = projected.reshape(-1, 2) - points
    rms = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))
    height, width = gray.shape[:2]
    margin = float(min(
        points[:, 0].min(),
        points[:, 1].min(),
        width - 1 - points[:, 0].max(),
        height - 1 - points[:, 1].max(),
    ))
    return {
        'rvec': [float(value) for value in rvec.reshape(3)],
        'tvec': [float(value) for value in tvec.reshape(3)],
        'rms_px': rms,
        'pixel_margin': margin,
        'corners': [[float(value) for value in point] for point in points],
        'marker_ids': [int(value) for value in ids.ravel()],
    }


def combine_detections(detections):
    """Combine repeated tag detections into one median sample."""
    valid = [tag for tag in detections if tag is not None]
    if not valid:
        return None
    translations = np.asarray(
        [tag['tvec'] for tag in valid], dtype=np.float64)
    median = np.median(translations, axis=0)
    distances = np.linalg.norm(translations - median, axis=1)
    best = int(np.argmin(distances))
    out = dict(valid[best])
    out['tvec'] = [float(value) for value in median]
    out['repeats'] = len(valid)
    out['translation_spread_m'] = float(distances.max())
    out['rms_px_max'] = float(max(tag['rms_px'] for tag in valid))
    out['repeat_tvec'] = [
        [float(value) for value in tag['tvec']] for tag in valid
    ]
    return out


class TagSurvey(Node):
    """Move the arm, capture frames and detect one AprilTag."""

    def __init__(self, output_dir, marker_id, size_m, duration,
                 settle=0.5, warmup=3, repeats=3, servo_reads=3):
        """Create subscriptions and the arm action client."""
        super().__init__('low_config_tag_survey')
        self.output_dir = output_dir
        self.marker_id = marker_id
        self.size_m = size_m
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
                and self.latest_image is not None, 15.0):
            raise RuntimeError(
                'joint state / camera info / image not received')

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

    def measured_arm_joints(self):
        """Return the five arm joints from the bus servos, or None."""
        try:
            pulses = self.sample_servo_pulses()
        except Exception as exc:  # noqa: BLE001 - fall back to commands
            print('initial servo read failed:', exc, flush=True)
            return None
        joints = pulses_to_joints(pulses)
        if len(joints) != len(ARM_JOINTS):
            return None
        return [float(joints[name]) for name in ARM_JOINTS]

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

    def capture(self, index, target, name):
        """Capture fresh frames and detect the tag."""
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
            detections.append(detect_tag(
                image, camera_matrix, dist_coeffs, self.marker_id,
                self.size_m))
        for repeat, image in enumerate(images):
            image_name = 'sample_{:02d}_{}_r{}.png'.format(
                index, name, repeat)
            cv2.imwrite(
                os.path.join(self.output_dir, image_name), image)
        tag = combine_detections(detections)
        servo_end = self.sample_servo_pulses()
        command_joints = self.arm_positions()
        start_joints = pulses_to_joints(servo_start)
        end_joints = pulses_to_joints(servo_end)
        measured_joints = [
            float(0.5 * (start_joints[key] + end_joints[key]))
            for key in ARM_JOINTS
        ]
        servo_shift = max(
            abs(servo_start[servo_id] - servo_end[servo_id])
            for servo_id in SERVO_IDS.values())
        return {
            'index': index,
            'name': name,
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
                float(start_joints[key]) for key in ARM_JOINTS
            ],
            'measured_joints_end': [
                float(end_joints[key]) for key in ARM_JOINTS
            ],
            'measured_joints': measured_joints,
            'servo_shift_pulses': int(servo_shift),
            'camera_frame': info.header.frame_id,
            'camera_matrix': camera_matrix.tolist(),
            'dist_coeffs': dist_coeffs.tolist(),
            'tag': tag,
            'image': 'sample_{:02d}_{}_r0.png'.format(index, name),
            'image_stamp': {
                'sec': self.latest_image.header.stamp.sec,
                'nanosec': self.latest_image.header.stamp.nanosec,
            },
        }


def load_poses(path):
    """Load named absolute five-joint targets from a pose-selection JSON."""
    with open(path, 'r', encoding='utf-8') as handle:
        data = json.load(handle)
    entries = data.get('poses') if isinstance(data, dict) else data
    if not isinstance(entries, list) or not entries:
        raise RuntimeError('{} has no non-empty poses list'.format(path))
    poses = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise RuntimeError('pose {} must be an object'.format(index))
        joints = entry.get('joints', entry.get('target_joints'))
        name = entry.get('name', 'pose{:02d}'.format(index))
        if joints is None or len(joints) != len(ARM_JOINTS):
            raise RuntimeError(
                'pose {} needs {} joint values'.format(
                    index, len(ARM_JOINTS)))
        values = [float(value) for value in joints]
        if not all(np.isfinite(values)):
            raise RuntimeError(
                'pose {} has non-finite joint values'.format(index))
        poses.append((name, values))
    return poses


def validate_target(target, label, margin=0.0):
    """Raise when a target is outside the adopted joint limits."""
    for name, value in zip(ARM_JOINTS, target):
        lower, upper = ARM_LIMITS[name]
        lower += margin
        upper -= margin
        if not lower <= value <= upper:
            raise RuntimeError(
                '{} {}={} outside [{}, {}]'.format(
                    label, name, value, lower, upper))


def clamp_target(target):
    """Clamp a return pose into the adopted joint limits."""
    out = []
    clamped = []
    for name, value in zip(ARM_JOINTS, target):
        lower, upper = ARM_LIMITS[name]
        bounded = min(max(value, lower), upper)
        if abs(bounded - value) > 1e-9:
            clamped.append('{} {:.4f}->{:.4f}'.format(name, value, bounded))
        out.append(bounded)
    return out, clamped


def approach_pose(target, offsets):
    """Return the pre-pose used to take up backlash before each target."""
    return [
        target[0] - offsets[0],
        target[1] + offsets[1],
        target[2] + offsets[2],
        target[3] + offsets[3],
        target[4],
    ]


def main():
    """Run the survey sequence."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output-dir', default='/tmp/low_tag_survey_20261119')
    parser.add_argument('--poses-file', required=True)
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-size-m', type=float, default=0.060)
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--settle', type=float, default=0.5)
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--servo-reads', type=int, default=3)
    parser.add_argument(
        '--pre-j1', type=float, default=0.08,
        help='pre-pose offset on joint1 (applied as target - offset)')
    parser.add_argument(
        '--pre-j2', type=float, default=0.05,
        help='pre-pose offset on joint2 (applied as target + offset)')
    parser.add_argument(
        '--pre-j3', type=float, default=0.05,
        help='pre-pose offset on joint3 (applied as target + offset)')
    parser.add_argument(
        '--pre-j4', type=float, default=0.0,
        help='pre-pose offset on joint4 (applied as target + offset)')
    parser.add_argument(
        '--check-only', action='store_true',
        help='capture at the current pose without moving')
    parser.add_argument('--test', action='store_true',
                        help='run only the first two poses')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    poses = load_poses(args.poses_file)
    if args.test:
        poses = poses[:2]
    offsets = (args.pre_j1, args.pre_j2, args.pre_j3, args.pre_j4)

    rclpy.init()
    node = TagSurvey(
        args.output_dir, args.marker_id, args.marker_size_m, args.duration,
        args.settle, args.warmup, args.repeats, args.servo_reads)
    samples = []
    start = None
    try:
        node.wait_ready()
        # /joint_states holds cached commands and is stale after a node
        # restart, so the return pose comes from the bus servos when possible.
        start = node.measured_arm_joints()
        if start is None:
            start = node.arm_positions()
            print('WARNING return pose from stale /joint_states', flush=True)
        else:
            print('START (measured)', [round(value, 5) for value in start],
                  flush=True)
        if args.check_only:
            samples.append(node.capture(0, start, 'check'))
        else:
            for index, (name, target) in enumerate(poses):
                validate_target(target, name)
                pre = approach_pose(target, offsets)
                validate_target(pre, name + ' pre-pose')
                print('PRE', name, [round(value, 4) for value in pre],
                      flush=True)
                node.move_to(pre, duration=max(6.0, 0.5 * args.duration))
                print('MOVE', name, [round(value, 4) for value in target],
                      flush=True)
                node.move_to(target, duration=max(6.0, args.duration))
                node.wait_positions(target)
                sample = node.capture(index, target, name)
                samples.append(sample)
                tag = sample['tag']
                if tag is None:
                    print('  {} tag NOT detected'.format(name), flush=True)
                else:
                    print(
                        '  {} tag ok rms={:.3f}px margin={:.0f}px '
                        'spread={:.2f}mm shift={}p tvec={}'.format(
                            name, tag['rms_px'],
                            tag.get('pixel_margin', float('nan')),
                            1000.0 * tag.get('translation_spread_m', 0.0),
                            sample.get('servo_shift_pulses', -1),
                            [round(value, 4) for value in tag['tvec']]),
                        flush=True)
    finally:
        if start is not None and not args.check_only:
            try:
                start, clamped = clamp_target(start)
                if clamped:
                    print('RETURN POSE CLAMPED', clamped, flush=True)
                print('RETURN TO START', flush=True)
                node.move_to(start, duration=max(8.0, args.duration))
            except Exception as exc:  # noqa: BLE001 - best-effort recovery
                print('return-to-start failed:', exc, flush=True)
        with open(os.path.join(args.output_dir, 'samples.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump({
                'marker_id': args.marker_id,
                'marker_size_m': args.marker_size_m,
                'duration_s': args.duration,
                'pre_pose_offsets_rad': {
                    'joint1': -args.pre_j1,
                    'joint2': args.pre_j2,
                    'joint3': args.pre_j3,
                    'joint4': args.pre_j4,
                },
                'poses_file': os.path.abspath(args.poses_file),
                'samples': samples,
            }, handle, indent=2)
        node.destroy_node()
        rclpy.shutdown()
        print('WROTE', os.path.join(args.output_dir, 'samples.json'),
              flush=True)


if __name__ == '__main__':
    main()
