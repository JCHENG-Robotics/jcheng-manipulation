"""
ROS 2 fiducial target localizer.

Subscribes to an image and ``CameraInfo``, detects a square fiducial, estimates
its 6D pose, and publishes ``target_pose``. The default output frame is the
camera optical frame from ``CameraInfo``; set ``output_frame`` to ``base_link``
to use TF.

The node does not move the robot. It is the perception input to the grasp
candidate search.
"""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped for TF2
from tf2_ros import Buffer, TransformListener

from .detector import DEFAULT_DICTIONARY, FiducialDetector


class FiducialLocalizer(Node):
    """Detect one fiducial marker and publish its pose."""

    def __init__(self):
        super().__init__('armpi_fiducial_localizer')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('marker_dictionary', DEFAULT_DICTIONARY)
        self.declare_parameter('marker_size_m', 0.06)
        self.declare_parameter('marker_id', -1)
        self.declare_parameter('output_frame', '')
        self.declare_parameter('max_reprojection_error_px', 3.0)
        self.declare_parameter('tf_timeout_s', 0.1)
        self.declare_parameter('publish_on_tf_failure', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).value
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.output_frame = str(
            self.get_parameter('output_frame').value
        ).strip()
        self.tf_timeout = float(
            self.get_parameter('tf_timeout_s').value
        )
        self.publish_on_tf_failure = bool(
            self.get_parameter('publish_on_tf_failure').value
        )
        self.detector = FiducialDetector(
            dictionary=self.get_parameter(
                'marker_dictionary'
            ).value,
            marker_size_m=self.get_parameter('marker_size_m').value,
            max_reprojection_error_px=self.get_parameter(
                'max_reprojection_error_px'
            ).value,
        )
        self.bridge = CvBridge()
        self.camera_info = None
        self.pose_publisher = self.create_publisher(
            PoseStamped, 'target_pose', 10
        )
        self.info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            10,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(
            'fiducial localizer: image={} camera_info={} output_frame={}'.format(
                self.image_topic,
                self.camera_info_topic,
                self.output_frame or '(camera)',
            )
        )

    def on_camera_info(self, message):
        """Store the latest camera intrinsics."""
        self.camera_info = message

    def on_image(self, message):
        """Detect a marker in one image and publish the selected pose."""
        if self.camera_info is None:
            self.get_logger().warn(
                'waiting for CameraInfo on {}'.format(
                    self.camera_info_topic
                ),
                throttle_duration_sec=5.0,
            )
            return
        image = self._to_gray(message)
        if image is None:
            return
        camera_matrix = np.asarray(
            self.camera_info.k, dtype=np.float64
        ).reshape(3, 3)
        detections = self.detector.detect(
            image, camera_matrix, self.camera_info.d
        )
        if self.marker_id >= 0:
            detections = [
                detection
                for detection in detections
                if detection.marker_id == self.marker_id
            ]
        if not detections:
            self.get_logger().debug('no fiducial detected')
            return
        detection = detections[0]
        pose = self._pose_stamped(message, detection)
        if self.output_frame and self.output_frame != pose.header.frame_id:
            try:
                pose = self.tf_buffer.transform(
                    pose,
                    self.output_frame,
                    timeout=Duration(seconds=self.tf_timeout),
                )
            except Exception as exc:  # noqa: BLE001 - report TF failure
                self.get_logger().warn(
                    'TF {} -> {} failed: {}'.format(
                        pose.header.frame_id, self.output_frame, exc
                    ),
                    throttle_duration_sec=5.0,
                )
                if not self.publish_on_tf_failure:
                    return
        self.pose_publisher.publish(pose)
        self.get_logger().info(
            'marker {} in {}: t=({:.3f},{:.3f},{:.3f}) '
            'reproj={:.2f}px'.format(
                detection.marker_id,
                pose.header.frame_id,
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                detection.reprojection_error_px,
            ),
            throttle_duration_sec=1.0,
        )

    def _to_gray(self, message):
        try:
            return self.bridge.imgmsg_to_cv2(
                message, desired_encoding='mono8'
            )
        except Exception:  # noqa: BLE001 - fall back to colour
            try:
                color = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding='bgr8'
                )
                return cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            except Exception as exc:  # noqa: BLE001 - report conversion
                self.get_logger().warn(
                    'image conversion failed: {}'.format(exc),
                    throttle_duration_sec=5.0,
                )
                return None

    def _pose_stamped(self, message, detection):
        pose = PoseStamped()
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        pose.header.stamp = stamp
        pose.header.frame_id = (
            message.header.frame_id
            or self.camera_info.header.frame_id
            or 'camera_optical_frame'
        )
        pose.pose.position.x = detection.tvec[0]
        pose.pose.position.y = detection.tvec[1]
        pose.pose.position.z = detection.tvec[2]
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = detection.quaternion
        return pose


def main(argv=None):
    """Run the fiducial localizer node."""
    rclpy.init(args=argv)
    node = FiducialLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
