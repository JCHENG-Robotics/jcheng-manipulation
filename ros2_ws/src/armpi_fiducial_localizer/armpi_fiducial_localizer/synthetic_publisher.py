"""
Publish a synthetic fiducial image and CameraInfo.

This utility lets the localizer be exercised without a camera. It renders a
marker at a known camera-frame pose and publishes ``image_raw`` and
``camera_info`` at a fixed rate. It does not emulate lens distortion, lighting
or motion blur.
"""

import argparse
import sys

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .synthetic import default_camera_matrix, render_scene


class SyntheticMarkerPublisher(Node):
    """Publish a static synthetic marker scene."""

    def __init__(self, args):
        super().__init__('synthetic_marker_publisher')
        self.args = args
        self.camera_matrix = default_camera_matrix(
            args.width, args.height, args.fx, args.fy
        )
        self.image, _ = render_scene(
            marker_id=args.marker_id,
            dictionary=args.marker_dictionary,
            marker_size_m=args.marker_size,
            camera_matrix=self.camera_matrix,
            rvec=tuple(args.rvec),
            tvec=tuple(args.tvec),
            image_size=(args.width, args.height),
            side_pixels=args.side_pixels,
            noise_sigma=args.noise_sigma,
        )
        self.bridge = CvBridge()
        self.camera_info = self._camera_info()
        self.image_publisher = self.create_publisher(
            Image, 'image_raw', 10
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, 'camera_info', 10
        )
        self.timer = self.create_timer(1.0 / args.rate, self.publish)
        self.get_logger().info(
            'synthetic marker {} at tvec={} rvec={}'.format(
                args.marker_id, tuple(args.tvec), tuple(args.rvec)
            )
        )

    def _camera_info(self):
        info = CameraInfo()
        info.width = self.args.width
        info.height = self.args.height
        info.distortion_model = 'plumb_bob'
        info.k = [float(value) for value in self.camera_matrix.reshape(-1)]
        info.d = [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        image_message = self.bridge.cv2_to_imgmsg(
            self.image, encoding='mono8'
        )
        image_message.header.stamp = stamp
        image_message.header.frame_id = self.args.frame_id
        self.image_publisher.publish(image_message)
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.args.frame_id
        self.info_publisher.publish(self.camera_info)


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Publish a synthetic fiducial scene.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--tvec',
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.4],
        metavar=('X', 'Y', 'Z'),
    )
    parser.add_argument(
        '--rvec',
        nargs=3,
        type=float,
        default=[3.141592653589793, 0.0, 0.0],
        metavar=('RX', 'RY', 'RZ'),
    )
    parser.add_argument('--marker-dictionary', default='DICT_APRILTAG_36h11')
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-size', type=float, default=0.06)
    parser.add_argument('--side-pixels', type=int, default=300)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fx', type=float, default=700.0)
    parser.add_argument('--fy', type=float, default=700.0)
    parser.add_argument('--noise-sigma', type=float, default=0.0)
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--frame-id', default='camera_optical_frame')
    return parser.parse_args(argv)


def main(argv=None):
    """Run the synthetic marker publisher."""
    raw_arguments = list(sys.argv[1:] if argv is None else argv)
    ros_arguments = []
    if '--ros-args' in raw_arguments:
        index = raw_arguments.index('--ros-args')
        ros_arguments = raw_arguments[index:]
        raw_arguments = raw_arguments[:index]
    args = _parse_args(raw_arguments)
    rclpy.init(args=ros_arguments)
    node = SyntheticMarkerPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
