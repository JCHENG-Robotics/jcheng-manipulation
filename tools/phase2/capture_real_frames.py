"""Capture real camera frames plus CameraInfo and the camera TF.

This runs inside the Humble container (CycloneDDS) and needs no OpenCV: each
frame is written as a binary PPM (P6) so the Jazzy-side detector can read it
with ``cv2.imread``. A JSON sidecar records the intrinsics, the image
timestamp and ``base_link -> rgb_camera_link`` at capture time.

Example:

    python3 capture_real_frames.py --output-dir /tmp/real_tag_20261118 \
        --count 5 --interval 0.4
"""

import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener


def _pack_rgb_rows(message):
    """Return tightly packed RGB bytes for a bgr8 or rgb8 image."""
    width = int(message.width)
    height = int(message.height)
    row_bytes = width * 3
    data = bytes(message.data)
    if int(message.step) != row_bytes:
        data = b''.join(
            data[row * message.step:row * message.step + row_bytes]
            for row in range(height)
        )
    if message.encoding == 'rgb8':
        return data
    if message.encoding != 'bgr8':
        raise ValueError(
            f'unsupported encoding {message.encoding!r}; expected bgr8/rgb8')
    swapped = bytearray(data)
    swapped[0::3], swapped[2::3] = swapped[2::3], swapped[0::3]
    return bytes(swapped)


def _write_ppm(path, width, height, rgb):
    """Write an 8-bit binary PPM (P6)."""
    with open(path, 'wb') as handle:
        handle.write(f'P6\n{width} {height}\n255\n'.encode('ascii'))
        handle.write(rgb)


def _transform_dict(transform):
    """Convert a tf2 transform into translation/quaternion/matrix."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    matrix = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
         translation.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
         translation.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
         translation.z],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return {
        'translation': [translation.x, translation.y, translation.z],
        'rotation_xyzw': [x, y, z, w],
        'matrix': matrix,
    }


class FrameCapture(Node):
    """Save N RGB frames with CameraInfo, TF and joint state."""

    def __init__(self, output_dir, count, interval, image_topic,
                 camera_info_topic, base_frame, camera_frame):
        super().__init__('real_frame_capture')
        self.output_dir = output_dir
        self.count = count
        self.interval = interval
        self.base_frame = base_frame
        self.camera_frame = camera_frame
        self.camera_info = None
        self.joint_positions = None
        self.saved = 0
        self.last_save = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo, camera_info_topic, self._info_cb, 10)
        self.create_subscription(
            Image, image_topic, self._image_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

    def _info_cb(self, message):
        self.camera_info = message

    def _joint_cb(self, message):
        self.joint_positions = dict(
            zip(message.name, message.position))

    def _image_cb(self, message):
        if self.camera_info is None or self.saved >= self.count:
            return
        now = time.monotonic()
        if now - self.last_save < self.interval:
            return
        try:
            rgb = _pack_rgb_rows(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, Time())
        except Exception as error:  # tf2 raises several exception types
            self.get_logger().warn(
                f'TF {self.base_frame}->{self.camera_frame} not ready: '
                f'{error}')
            return

        index = self.saved
        stem = os.path.join(self.output_dir, f'frame_{index:02d}')
        _write_ppm(
            stem + '.ppm', int(message.width), int(message.height), rgb)
        metadata = {
            'frame_id': message.header.frame_id,
            'stamp': {
                'sec': message.header.stamp.sec,
                'nanosec': message.header.stamp.nanosec,
            },
            'width': int(message.width),
            'height': int(message.height),
            'encoding': message.encoding,
            'step': int(message.step),
            'camera_info': {
                'frame_id': self.camera_info.header.frame_id,
                'width': int(self.camera_info.width),
                'height': int(self.camera_info.height),
                'distortion_model': self.camera_info.distortion_model,
                'k': [float(value) for value in self.camera_info.k],
                'd': [float(value) for value in self.camera_info.d],
                'r': [float(value) for value in self.camera_info.r],
                'p': [float(value) for value in self.camera_info.p],
            },
            'tf_base_camera': _transform_dict(transform),
            'joint_positions': self.joint_positions or {},
        }
        with open(stem + '.json', 'w', encoding='utf-8') as handle:
            json.dump(metadata, handle, indent=2)
        self.saved += 1
        self.last_save = now
        self.get_logger().info(
            f'saved {stem}.ppm ({message.width}x{message.height} '
            f'{message.encoding})')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--count', type=int, default=5)
    parser.add_argument('--interval', type=float, default=0.4)
    parser.add_argument('--image-topic', default='/depth_cam/rgb/image_raw')
    parser.add_argument(
        '--camera-info-topic', default='/depth_cam/rgb/camera_info')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--camera-frame', default='rgb_camera_link')
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rclpy.init()
    node = FrameCapture(
        args.output_dir, args.count, args.interval, args.image_topic,
        args.camera_info_topic, args.base_frame, args.camera_frame)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.saved < args.count:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() > deadline:
                node.get_logger().error(
                    f'timeout: saved {node.saved}/{args.count} frames')
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'saved {node.saved} frames to {args.output_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
