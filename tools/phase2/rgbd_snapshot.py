#!/usr/bin/env python3
"""
Capture a same-scene RGB frame, cloud, camera info and joints.

This is the container-side half of the same-frame RGB-D / chessboard check. The
Humble container has numpy but **no OpenCV/PIL**, so the arrays are saved as
``.npy`` and the host analyses them. The node only subscribes; it never
publishes or moves the robot.

Run inside the container:

    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    python3 /repo_tools/phase2/rgbd_snapshot.py \
        --output-dir /tmp/rgbd_board_20261118 --max-skew 0.20

Output: ``rgb.npy``, ``points.npy``, ``metadata.json`` (stamps, encodings,
camera info, joint names/positions). Copy the directory to the host for the
OpenCV/PnP analysis.
"""

import argparse
from datetime import datetime, timezone
import json
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2

RGB_TOPIC = '/depth_cam/rgb/image_raw'
RGB_INFO_TOPIC = '/depth_cam/rgb/camera_info'
DEPTH_INFO_TOPIC = '/depth_cam/depth/camera_info'
DEPTH_RAW_TOPIC = '/depth_cam/depth/image_raw'
IR_TOPIC = '/depth_cam/ir/image_raw'
CLOUD_TOPIC = '/depth_cam/depth_registered/points'
JOINT_TOPIC = '/joint_states'


def image_to_array(message):
    """Convert a sensor_msgs/Image to a numpy array without cv_bridge."""
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1, 'mono16': 1, '16UC1': 1}
    if message.encoding not in channels:
        raise ValueError('unsupported image encoding {}'.format(
            message.encoding))
    count = channels[message.encoding]
    dtype = np.uint16 if message.encoding in ('mono16', '16UC1') else np.uint8
    itemsize = np.dtype(dtype).itemsize
    row_items = message.step // itemsize
    array = np.frombuffer(message.data, dtype=dtype).reshape(
        message.height, row_items)
    array = array[:, :message.width * count]
    if count == 1:
        return array.reshape(message.height, message.width).copy()
    return array.reshape(message.height, message.width, count).copy()


def cloud_to_array(message):
    """Convert a PointCloud2 x/y/z cloud to an Nx3 float32 array."""
    fields = {field.name: field for field in message.fields}
    missing = {'x', 'y', 'z'} - set(fields)
    if missing:
        raise ValueError('cloud is missing fields {}'.format(sorted(missing)))
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [np.float32, np.float32, np.float32],
        'offsets': [fields['x'].offset, fields['y'].offset,
                    fields['z'].offset],
        'itemsize': message.point_step,
    })
    data = np.frombuffer(message.data, dtype=dtype)
    points = np.stack([data['x'], data['y'], data['z']], axis=1)
    return points.astype(np.float32).copy()


def camera_info_dict(message):
    """Return a JSON-friendly CameraInfo summary."""
    return {
        'frame_id': message.header.frame_id,
        'width': int(message.width),
        'height': int(message.height),
        'distortion_model': message.distortion_model,
        'k': [float(value) for value in message.k],
        'd': [float(value) for value in message.d],
        'r': [float(value) for value in message.r],
        'p': [float(value) for value in message.p],
    }


def stamp_dict(stamp):
    """Return a JSON-friendly ROS stamp."""
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


class RgbdSnapshot(Node):
    """Collect one RGB frame, one registered cloud and the current joints."""

    def __init__(self, output_dir, max_skew):
        """Subscribe to the camera, cloud and joint-state topics."""
        super().__init__('phase2_rgbd_snapshot')
        self.output_dir = output_dir
        self.max_skew = max_skew
        self.latest = {}
        self.received = {}
        self.camera_info = {}
        self.saved = False
        os.makedirs(output_dir, exist_ok=True)
        self.create_subscription(
            Image, RGB_TOPIC, self._rgb_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, RGB_INFO_TOPIC, self._rgb_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, DEPTH_INFO_TOPIC, self._depth_info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, CLOUD_TOPIC, self._cloud_cb, qos_profile_sensor_data)
        self.create_subscription(
            Image, DEPTH_RAW_TOPIC, self._depth_raw_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, IR_TOPIC, self._ir_cb, qos_profile_sensor_data)
        self.create_subscription(
            JointState, JOINT_TOPIC, self._joint_cb, 10)

    def _stamp(self, message):
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _rgb_cb(self, message):
        self.latest['rgb'] = message
        self.received['rgb'] = self.get_clock().now().nanoseconds * 1e-9

    def _rgb_info_cb(self, message):
        self.camera_info['rgb'] = message

    def _depth_info_cb(self, message):
        self.camera_info['depth'] = message

    def _cloud_cb(self, message):
        self.latest['cloud'] = message
        self.received['cloud'] = self.get_clock().now().nanoseconds * 1e-9
        self._maybe_save()

    def _depth_raw_cb(self, message):
        self.latest['depth_raw'] = message

    def _ir_cb(self, message):
        self.latest['ir'] = message

    def _joint_cb(self, message):
        self.latest['joints'] = message
        self.received['joints'] = self.get_clock().now().nanoseconds * 1e-9

    def _maybe_save(self):
        if self.saved:
            return
        if 'rgb' not in self.latest or 'joints' not in self.latest:
            return
        if 'rgb' not in self.camera_info or 'depth' not in self.camera_info:
            return
        if 'depth_raw' not in self.latest or 'ir' not in self.latest:
            return
        rgb_stamp = self._stamp(self.latest['rgb'])
        cloud_stamp = self._stamp(self.latest['cloud'])
        joint_age = (
            self.get_clock().now().nanoseconds * 1e-9
            - self.received['joints']
        )
        if abs(rgb_stamp - cloud_stamp) > self.max_skew:
            return
        if joint_age > 0.5:
            return
        self.saved = True
        self._write()

    def _write(self):
        rgb_message = self.latest['rgb']
        cloud_message = self.latest['cloud']
        joint_message = self.latest['joints']
        rgb = image_to_array(rgb_message)
        points = cloud_to_array(cloud_message)
        np.save(os.path.join(self.output_dir, 'rgb.npy'), rgb)
        np.save(os.path.join(self.output_dir, 'points.npy'), points)
        raw_images = {}
        for key, filename, topic in (
            ('depth_raw', 'depth.npy', DEPTH_RAW_TOPIC),
            ('ir', 'ir.npy', IR_TOPIC),
        ):
            message = self.latest.get(key)
            if message is None:
                continue
            np.save(
                os.path.join(self.output_dir, filename),
                image_to_array(message),
            )
            raw_images[key] = {
                'topic': topic,
                'encoding': message.encoding,
                'width': int(message.width),
                'height': int(message.height),
                'frame_id': message.header.frame_id,
                'stamp': stamp_dict(message.header.stamp),
            }
        finite = np.isfinite(points).all(axis=1)
        metadata = {
            'schema_version': 1,
            'generated_at': datetime.now(timezone.utc).isoformat(),
            'mode': 'same_scene_rgbd_snapshot',
            'motion': 'none',
            'max_skew_s': self.max_skew,
            'rgb': {
                'topic': RGB_TOPIC,
                'encoding': rgb_message.encoding,
                'width': int(rgb_message.width),
                'height': int(rgb_message.height),
                'frame_id': rgb_message.header.frame_id,
                'stamp': stamp_dict(rgb_message.header.stamp),
                'camera_info': (
                    camera_info_dict(self.camera_info['rgb'])
                    if 'rgb' in self.camera_info
                    else None
                ),
            },
            'cloud': {
                'topic': CLOUD_TOPIC,
                'width': int(cloud_message.width),
                'height': int(cloud_message.height),
                'frame_id': cloud_message.header.frame_id,
                'stamp': stamp_dict(cloud_message.header.stamp),
                'points': int(len(points)),
                'finite_points': int(np.count_nonzero(finite)),
                'depth_camera_info': (
                    self.camera_info.get('depth')
                    and camera_info_dict(self.camera_info['depth'])
                ),
            },
            'joints': {
                'topic': JOINT_TOPIC,
                'names': list(joint_message.name),
                'positions': [
                    float(value) for value in joint_message.position
                ],
                'stamp': stamp_dict(joint_message.header.stamp),
                'received_at_s': float(self.received['joints']),
            },
            'raw_images': raw_images,
        }
        with open(
            os.path.join(self.output_dir, 'metadata.json'),
            'w', encoding='utf-8',
        ) as handle:
            json.dump(metadata, handle, indent=2)
        self.get_logger().info(
            'saved RGB {}x{} and {} cloud points to {}'.format(
                rgb.shape[1], rgb.shape[0], len(points), self.output_dir
            )
        )

    def wait_and_save(self, timeout):
        """Spin until a synchronized snapshot has been written."""
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while (
            not self.saved
            and self.get_clock().now().nanoseconds * 1e-9 < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.saved


def main(argv=None):
    """Capture one snapshot. Returns a process exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--max-skew', type=float, default=0.20)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args(argv)
    rclpy.init(args=[])
    node = RgbdSnapshot(args.output_dir, args.max_skew)
    try:
        if not node.wait_and_save(args.timeout):
            raise SystemExit(
                'no synchronized RGB/cloud snapshot before timeout')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
