#!/usr/bin/env python3
"""Publish the off-board front camera's static TF and camera_info.

The RealSense D435i is fixed to the chassis front, so ``base_link ->
ext_cam_optical`` is a constant solved once (2026-11-20) and published as a
static transform.  The intrinsics live in the same calibration report, so this
node is the single place that turns the two JSON files into the TF tree and a
``camera_info`` consumers can latch.

The node deliberately does **not** open the camera: the studio gateway already
owns ``/dev/video4`` for its aux-camera preview.  When a ROS image path is
wanted, one owner has to be chosen first.

Run inside the robot container:

    python3 /tmp/ext_cam_frames.py \
      --extrinsic /tmp/front_camera_extrinsic.json \
      --intrinsics /tmp/uvc_color_intrinsics_aspect.json
"""

import argparse
import json
import math

PARENT_FRAME = 'base_link'
CHILD_FRAME = 'ext_cam_optical'
CAMERA_INFO_TOPIC = '/ext_cam/color/camera_info'


def rotation_matrix_to_quaternion(rotation):
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion."""
    m = [[float(rotation[i][j]) for j in range(3)] for i in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def load_calibration(extrinsic_path, intrinsics_path):
    """Read the two reports and return the fields the node publishes."""
    with open(extrinsic_path) as handle:
        extrinsic = json.load(handle)
    with open(intrinsics_path) as handle:
        intrinsics = json.load(handle)
    transform = extrinsic['T_base_front']
    return {
        'translation': [float(transform[i][3]) for i in range(3)],
        'rotation': [row[:3] for row in transform[:3]],
        'camera_matrix': intrinsics['camera_matrix'],
        'dist_coeffs': intrinsics['dist_coeffs'],
        'width': int(intrinsics['image_size'][0]),
        'height': int(intrinsics['image_size'][1]),
        'model': intrinsics.get('model', 'plumb_bob'),
    }


def camera_info_fields(calibration, frame_id=CHILD_FRAME):
    """Return the CameraInfo field values as plain Python data."""
    matrix = calibration['camera_matrix']
    k = [float(value) for row in matrix for value in row]
    fx, fy = k[0], k[4]
    cx, cy = k[2], k[5]
    return {
        'frame_id': frame_id,
        'width': int(calibration['width']),
        'height': int(calibration['height']),
        'distortion_model': calibration['model'],
        'd': [float(value) for value in calibration['dist_coeffs']],
        'k': k,
        # Plumb-bob P: K with zero translation, the sensor_msgs convention for
        # a monocular camera.
        'p': [fx, 0.0, cx, 0.0,
              0.0, fy, cy, 0.0,
              0.0, 0.0, 1.0, 0.0],
    }


def build_camera_info(calibration, frame_id=CHILD_FRAME):
    """Return a CameraInfo message populated from the calibration report."""
    from sensor_msgs.msg import CameraInfo

    fields = camera_info_fields(calibration, frame_id)
    info = CameraInfo()
    info.header.frame_id = fields['frame_id']
    info.width = fields['width']
    info.height = fields['height']
    info.distortion_model = fields['distortion_model']
    info.d = fields['d']
    info.k = fields['k']
    info.p = fields['p']
    return info


def build_transform(calibration, parent=PARENT_FRAME, child=CHILD_FRAME):
    """Return the static TransformStamped for base_link -> optical frame."""
    from geometry_msgs.msg import TransformStamped

    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x, \
        message.transform.translation.y, \
        message.transform.translation.z = calibration['translation']
    quaternion = rotation_matrix_to_quaternion(calibration['rotation'])
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish the front camera static TF and camera_info.')
    parser.add_argument('--extrinsic', required=True)
    parser.add_argument('--intrinsics', required=True)
    parser.add_argument('--camera-info-topic', default=CAMERA_INFO_TOPIC)
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from tf2_ros.static_transform_broadcaster import \
        StaticTransformBroadcaster

    calibration = load_calibration(args.extrinsic, args.intrinsics)
    rclpy.init()
    node = Node('ext_cam_frames')
    broadcaster = StaticTransformBroadcaster(node)
    message = build_transform(calibration)
    message.header.stamp = node.get_clock().now().to_msg()
    broadcaster.sendTransform(message)
    info = build_camera_info(calibration)
    info.header.stamp = message.header.stamp
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(CameraInfoType(), args.camera_info_topic,
                                      qos)
    publisher.publish(info)
    node.get_logger().info(
        'published {} -> {} and {} ({}x{})'.format(
            PARENT_FRAME, CHILD_FRAME, args.camera_info_topic,
            calibration['width'], calibration['height']))
    rclpy.spin(node)


def CameraInfoType():
    """Import CameraInfo lazily so the helpers stay testable without ROS."""
    from sensor_msgs.msg import CameraInfo
    return CameraInfo


if __name__ == '__main__':
    main()
