#!/usr/bin/env python3
"""Measure live camera image rates and optionally save one RGB frame.

Run this inside the robot container, where the vendor camera driver runs:

    sudo docker cp tools/phase1/camera_stream_check.py ArmPiUltra:/tmp/
    sudo docker exec -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; \
       python3 /tmp/camera_stream_check.py --seconds 5 --save-rgb /tmp/rgb_frame.png'

The camera publishes sensor-data QoS (best effort), hence
``qos_profile_sensor_data``. The vendor Aurora 930 publishes both RGB and depth
streams; the depth values are millimetres in a ``mono16`` image.
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class CameraStreamCheck(Node):
    """Count RGB and depth images until the measurement window closes."""

    def __init__(self) -> None:
        """Create the two sensor-QoS subscriptions."""
        super().__init__('camera_stream_check')
        self.rgb = []
        self.depth = []
        self.create_subscription(
            Image, '/depth_cam/rgb/image_raw', self._rgb_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/depth_cam/depth/image_raw', self._depth_cb,
            qos_profile_sensor_data)

    def _rgb_cb(self, msg: Image) -> None:
        self.rgb.append(msg)

    def _depth_cb(self, msg: Image) -> None:
        self.depth.append(msg)


def _save_rgb(msg: Image, path: str) -> None:
    """Save an 8-bit RGB/BGR image without requiring cv_bridge."""
    import cv2
    import numpy as np

    if msg.encoding not in ('bgr8', 'rgb8'):
        print(f'not saving unsupported encoding {msg.encoding}')
        return
    row_bytes = msg.width * 3
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    arr = arr.reshape(msg.height, msg.step)[:, :row_bytes]
    arr = arr.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, arr)
    print(f'saved {path}')


def main() -> None:
    """Run the measurement."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--save-rgb', default='')
    args = parser.parse_args()

    rclpy.init()
    node = CameraStreamCheck()
    end = time.time() + args.seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)

    for name, msgs in (('rgb', node.rgb), ('depth', node.depth)):
        if not msgs:
            print(f'{name}: no messages in {args.seconds:.1f}s')
            continue
        msg = msgs[-1]
        rate = len(msgs) / args.seconds
        print(
            f'{name}: {len(msgs)} msgs in {args.seconds:.1f}s '
            f'({rate:.1f} Hz), {msg.width}x{msg.height} '
            f'encoding={msg.encoding} step={msg.step} '
            f'frame={msg.header.frame_id}')
        if name == 'depth' and msg.encoding == 'mono16':
            import numpy as np

            arr = np.frombuffer(msg.data, dtype=np.uint16)
            arr = arr.reshape(msg.height, msg.step // 2)[:, :msg.width]
            print(
                f'  depth values: min={arr.min()} max={arr.max()} '
                f'median={np.median(arr):.1f} zeros={float((arr == 0).mean()):.2%}')
        if name == 'rgb' and args.save_rgb:
            try:
                _save_rgb(msg, args.save_rgb)
            except Exception as exc:  # noqa: BLE001 - best-effort evidence capture
                print(f'save failed: {exc}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
