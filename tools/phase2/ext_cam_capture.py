#!/usr/bin/env python3
"""Capture one off-board front-camera colour frame and one depth frame.

Run inside the robot container (no OpenCV there): the colour stream is the UVC
topic owned by ``armpi-ext-cam`` and the depth stream is the RealSense node
owned by ``armpi-rs-depth``. Both are only subscribed; the tool never opens the
devices and never moves the robot.

    python3 /tmp/ext_cam_capture.py --output /tmp/ext_cam_frame

Output: ``color.npy`` (H x W x 3 BGR), ``depth.npy`` (H x W uint16
millimetres) and ``metadata.json`` with both stamps, encodings, camera_info and
the measured frame skew. The scene is static, so a small skew is harmless; it
is recorded so a moving scene can be rejected.
"""

import argparse
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

COLOR_TOPIC = '/ext_cam/color/image_raw'
COLOR_INFO_TOPIC = '/ext_cam/color/camera_info'
DEPTH_TOPIC = '/camera/camera/depth/image_rect_raw'
DEPTH_INFO_TOPIC = '/camera/camera/depth/camera_info'


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


def stamp_seconds(message):
    """Return the header stamp as float seconds."""
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


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


class Capture(Node):
    """One colour frame and one depth frame, with their camera_info."""

    def __init__(self, color_topic, color_info_topic, depth_topic,
                 depth_info_topic):
        super().__init__('ext_cam_capture')
        self.color = None
        self.color_info = None
        self.depth = None
        self.depth_info = None
        qos = qos_profile_sensor_data
        self.create_subscription(
            Image, color_topic, self.on_color, qos)
        self.create_subscription(
            CameraInfo, color_info_topic, self.on_color_info, qos)
        self.create_subscription(
            Image, depth_topic, self.on_depth, qos)
        self.create_subscription(
            CameraInfo, depth_info_topic, self.on_depth_info, qos)

    def on_color(self, message):
        """Keep the latest colour frame."""
        self.color = (image_to_array(message), stamp_seconds(message),
                      message.header.frame_id, message.encoding)

    def on_color_info(self, message):
        """Keep the latest colour camera_info."""
        self.color_info = camera_info_dict(message)

    def on_depth(self, message):
        """Keep the latest depth frame."""
        self.depth = (image_to_array(message), stamp_seconds(message),
                      message.header.frame_id, message.encoding)

    def on_depth_info(self, message):
        """Keep the latest depth camera_info."""
        self.depth_info = camera_info_dict(message)


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True,
                        help='output directory (created if missing)')
    parser.add_argument('--color-topic', default=COLOR_TOPIC)
    parser.add_argument('--color-info-topic', default=COLOR_INFO_TOPIC)
    parser.add_argument('--depth-topic', default=DEPTH_TOPIC)
    parser.add_argument('--depth-info-topic', default=DEPTH_INFO_TOPIC)
    parser.add_argument('--timeout', type=float, default=20.0,
                        help='seconds to wait for both frames')
    parser.add_argument('--max-skew', type=float, default=1.0,
                        help='refuse when the two stamps differ by more')
    parser.add_argument('--max-age', type=float, default=10.0,
                        help='refuse when a frame stamp is older than this '
                             'many seconds; the colour stream can lag while '
                             'the depth node shares the USB link')
    return parser


def main(argv=None):
    """Capture and write the three files."""
    args = build_parser().parse_args(argv)
    os.makedirs(args.output, exist_ok=True)
    rclpy.init()
    node = Capture(
        args.color_topic, args.color_info_topic,
        args.depth_topic, args.depth_info_topic)
    try:
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (node.color is not None and node.color_info is not None
                    and node.depth is not None
                    and node.depth_info is not None):
                break
        missing = [
            name for name, value in (
                ('color', node.color), ('color_info', node.color_info),
                ('depth', node.depth), ('depth_info', node.depth_info))
            if value is None
        ]
        if missing:
            raise SystemExit('missing frames: {}'.format(', '.join(missing)))
        captured_at = time.time()
        color, color_stamp, color_frame, color_encoding = node.color
        depth, depth_stamp, depth_frame, depth_encoding = node.depth
        skew = abs(color_stamp - depth_stamp)
        if skew > args.max_skew:
            raise SystemExit(
                'frame skew {:.3f} s exceeds --max-skew {:.3f}'.format(
                    skew, args.max_skew))
        color_age = captured_at - color_stamp
        depth_age = captured_at - depth_stamp
        if max(color_age, depth_age) > args.max_age:
            raise SystemExit(
                'stale frame: colour {:.2f} s, depth {:.2f} s old exceeds '
                '--max-age {:.2f}'.format(
                    color_age, depth_age, args.max_age))
        np.save(os.path.join(args.output, 'color.npy'), color)
        np.save(os.path.join(args.output, 'depth.npy'), depth)
        metadata = {
            'color': {
                'topic': args.color_topic,
                'stamp': color_stamp,
                'frame_id': color_frame,
                'encoding': color_encoding,
                'shape': list(color.shape),
                'camera_info': node.color_info,
            },
            'depth': {
                'topic': args.depth_topic,
                'stamp': depth_stamp,
                'frame_id': depth_frame,
                'encoding': depth_encoding,
                'shape': list(depth.shape),
                'camera_info': node.depth_info,
                'scale_m_per_unit': 0.001,
                'valid_fraction': float((depth > 0).mean()),
                'median_m': float(np.median(depth[depth > 0])) * 0.001
                if (depth > 0).any() else None,
            },
            'captured_at': captured_at,
            'skew_s': float(skew),
            'age_s': {
                'color': float(color_age),
                'depth': float(depth_age),
            },
        }
        with open(os.path.join(args.output, 'metadata.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump(metadata, handle, indent=2)
        print('colour {} {} skew {:.3f} s'.format(
            color.shape, color_encoding, skew))
        print('depth {} {} valid {:.1%} median {:.3f} m'.format(
            depth.shape, depth_encoding,
            metadata['depth']['valid_fraction'],
            metadata['depth']['median_m'] or 0.0))
        print('WROTE', args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
