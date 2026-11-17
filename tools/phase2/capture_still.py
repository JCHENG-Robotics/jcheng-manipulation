#!/usr/bin/env python3
"""Capture still RGB/depth frames and TF snapshots without moving the robot.

Run this inside the robot container (it has rclpy and the vendor camera
driver). The script only subscribes; it never publishes a servo command. It is
intended for offline perception tests and frame-contract checks:

    sudo docker cp tools/phase2/capture_still.py ArmPiUltra:/tmp/
    sudo docker exec -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; exec python3 /tmp/capture_still.py \
         --output-dir /tmp/real_capture_20261117 --count 8 \
         --tf link4:rgb_camera_link --tf link4:depth_camera_link'

Raw images stay in the output directory. Do not commit frames that contain
people or private workspace details.
"""

import argparse
from datetime import datetime, timezone
import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

try:
    import tf2_ros
except ImportError:  # pragma: no cover - tf2 is present in the container
    tf2_ros = None


RGB_TOPIC = '/depth_cam/rgb/image_raw'
RGB_INFO_TOPIC = '/depth_cam/rgb/camera_info'
DEPTH_TOPIC = '/depth_cam/depth/image_raw'
DEPTH_INFO_TOPIC = '/depth_cam/depth/camera_info'


def image_to_array(msg):
    """Convert a sensor_msgs/Image to a numpy array without cv_bridge."""
    channels = {
        'bgr8': 3,
        'rgb8': 3,
        'mono8': 1,
        'mono16': 1,
        '16UC1': 1,
    }
    if msg.encoding not in channels:
        raise ValueError('unsupported image encoding {}'.format(msg.encoding))
    count = channels[msg.encoding]
    dtype = np.uint16 if msg.encoding in ('mono16', '16UC1') else np.uint8
    row_bytes = msg.width * count * np.dtype(dtype).itemsize
    array = np.frombuffer(msg.data, dtype=dtype).reshape(
        msg.height, msg.step // np.dtype(dtype).itemsize)
    array = array[:, :row_bytes // np.dtype(dtype).itemsize]
    if count == 1:
        return array.reshape(msg.height, msg.width).copy()
    return array.reshape(msg.height, msg.width, count).copy()


def camera_info_dict(msg):
    """Return a JSON-friendly CameraInfo summary."""
    return {
        'frame_id': msg.header.frame_id,
        'width': int(msg.width),
        'height': int(msg.height),
        'distortion_model': msg.distortion_model,
        'k': [float(value) for value in msg.k],
        'd': [float(value) for value in msg.d],
        'r': [float(value) for value in msg.r],
        'p': [float(value) for value in msg.p],
    }


class StillCapture(Node):
    """Subscribe to the camera and record a fixed number of frames."""

    def __init__(self, output_dir, count, interval, capture_depth, tf_pairs):
        """Create the subscriptions and the output directory."""
        super().__init__('phase2_capture_still')
        self.output_dir = output_dir
        self.count = count
        self.interval = interval
        self.capture_depth = capture_depth
        self.tf_pairs = tf_pairs
        self.latest = {}
        self.camera_info = {}
        self.saved = []
        self.last_save = 0.0
        os.makedirs(output_dir, exist_ok=True)

        self.create_subscription(
            Image, RGB_TOPIC, self._rgb_cb, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, RGB_INFO_TOPIC, self._rgb_info_cb,
            qos_profile_sensor_data)
        if capture_depth:
            self.create_subscription(
                Image, DEPTH_TOPIC, self._depth_cb, qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, DEPTH_INFO_TOPIC, self._depth_info_cb,
                qos_profile_sensor_data)

        self.tf_buffer = None
        self.tf_listener = None
        if tf_pairs and tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self)

    def _rgb_cb(self, msg):
        self.latest['rgb'] = msg

    def _rgb_info_cb(self, msg):
        self.camera_info['rgb'] = msg

    def _depth_cb(self, msg):
        self.latest['depth'] = msg

    def _depth_info_cb(self, msg):
        self.camera_info['depth'] = msg

    def _ready(self):
        if 'rgb' not in self.latest or 'rgb' not in self.camera_info:
            return False
        if self.capture_depth and (
                'depth' not in self.latest
                or 'depth' not in self.camera_info):
            return False
        return True

    def _tf_snapshot(self):
        if self.tf_buffer is None:
            return {}
        snapshot = {}
        for parent, child in self.tf_pairs:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                snapshot['{}->{}'.format(parent, child)] = {
                    'parent': parent,
                    'child': child,
                    'translation_m': [
                        float(translation.x),
                        float(translation.y),
                        float(translation.z),
                    ],
                    'quaternion_xyzw': [
                        float(rotation.x),
                        float(rotation.y),
                        float(rotation.z),
                        float(rotation.w),
                    ],
                    'stamp': {
                        'sec': int(transform.header.stamp.sec),
                        'nanosec': int(transform.header.stamp.nanosec),
                    },
                }
            except Exception as exc:  # noqa: BLE001 - report TF failures
                snapshot['{}->{}'.format(parent, child)] = {
                    'error': str(exc),
                }
        return snapshot

    def capture(self, timeout=20.0):
        """Wait for the streams and save frames at the requested interval."""
        deadline = time.monotonic() + timeout
        while not self._ready() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self._ready():
            raise RuntimeError('camera streams not ready before timeout')

        frames = []
        while len(frames) < self.count and time.monotonic() < deadline:
            now = time.monotonic()
            if now - self.last_save >= self.interval:
                index = len(frames)
                rgb_msg = self.latest['rgb']
                rgb = image_to_array(rgb_msg)
                rgb_path = os.path.join(
                    self.output_dir, 'rgb_{:03d}.png'.format(index))
                if not cv2.imwrite(rgb_path, rgb):
                    raise RuntimeError('failed to write {}'.format(rgb_path))
                entry = {
                    'index': index,
                    'path': os.path.basename(rgb_path),
                    'encoding': rgb_msg.encoding,
                    'width': int(rgb_msg.width),
                    'height': int(rgb_msg.height),
                    'frame_id': rgb_msg.header.frame_id,
                    'stamp': {
                        'sec': int(rgb_msg.header.stamp.sec),
                        'nanosec': int(rgb_msg.header.stamp.nanosec),
                    },
                    'mean': float(np.mean(rgb)),
                    'min': int(np.min(rgb)),
                    'max': int(np.max(rgb)),
                    'rgb': rgb_path,
                    'camera_info': camera_info_dict(self.camera_info['rgb']),
                }
                if self.capture_depth:
                    depth_msg = self.latest['depth']
                    depth = image_to_array(depth_msg)
                    depth_path = os.path.join(
                        self.output_dir,
                        'depth_{:03d}.png'.format(index))
                    if not cv2.imwrite(depth_path, depth):
                        raise RuntimeError(
                            'failed to write {}'.format(depth_path))
                    entry.update({
                        'depth_path': os.path.basename(depth_path),
                        'depth_encoding': depth_msg.encoding,
                        'depth_frame_id': depth_msg.header.frame_id,
                        'depth_min': int(np.min(depth)),
                        'depth_max': int(np.max(depth)),
                        'depth_median': float(np.median(depth)),
                        'depth_zero_fraction': float(np.mean(depth == 0)),
                        'depth_camera_info': camera_info_dict(
                            self.camera_info['depth']),
                    })
                frames.append(entry)
                self.last_save = now
            rclpy.spin_once(self, timeout_sec=0.02)
        if len(frames) < self.count:
            raise RuntimeError(
                'only {} of {} frames captured'.format(
                    len(frames), self.count))
        return frames

    def write_metadata(self, frames):
        """Write the capture metadata and the TF snapshot."""
        metadata = {
            'schema_version': 1,
            'generated_at': datetime.now(timezone.utc).isoformat(),
            'mode': 'real_camera_still_capture',
            'motion': 'none',
            'topics': {
                'rgb': RGB_TOPIC,
                'rgb_info': RGB_INFO_TOPIC,
                'depth': DEPTH_TOPIC if self.capture_depth else None,
                'depth_info': DEPTH_INFO_TOPIC if self.capture_depth else None,
            },
            'frames': frames,
            'tf': self._tf_snapshot(),
            'private': True,
        }
        path = os.path.join(self.output_dir, 'capture.json')
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(metadata, handle, indent=2)
        return path


def _parse_tf(value):
    parent, _, child = value.partition(':')
    if not child:
        raise argparse.ArgumentTypeError(
            'expected parent:child, got {!r}'.format(value))
    return parent, child


def main():
    """Run the still capture."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--count', type=int, default=8)
    parser.add_argument('--interval', type=float, default=0.4)
    parser.add_argument('--timeout', type=float, default=20.0)
    parser.add_argument('--depth', action='store_true')
    parser.add_argument(
        '--tf', action='append', type=_parse_tf, default=[],
        metavar='PARENT:CHILD')
    args = parser.parse_args()

    rclpy.init()
    node = StillCapture(
        args.output_dir, args.count, args.interval, args.depth, args.tf)
    try:
        frames = node.capture(timeout=args.timeout)
        path = node.write_metadata(frames)
        print('CAPTURED {} frames -> {}'.format(len(frames), path))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
