"""
Publish the measured body -> optical camera frame as a static TF.

The vendor RGB `CameraInfo` is tagged `rgb_camera_link`, but OpenCV PnP
returns the marker pose in the optical frame. Publishing the measured optical
frame under `rgb_camera_link` gives TF and perception code one unambiguous
name: `camera_optical_calibrated`.

The transform is loaded from `config/camera_optical_frame.yaml`, which is
generated from the validated chessboard hand-eye and the full-precision live
body-frame TF. This node does not move the robot.
"""

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .frames import default_config_path, load_frame_config


def transform_message(config):
    """Build a TransformStamped from a frame-contract dictionary."""
    transform = TransformStamped()
    transform.header.frame_id = str(config['parent_frame'])
    transform.child_frame_id = str(config['child_frame'])
    transform.transform.translation.x = config['translation_m'][0]
    transform.transform.translation.y = config['translation_m'][1]
    transform.transform.translation.z = config['translation_m'][2]
    (
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ) = config['quaternion_xyzw']
    return transform


class CameraFramePublisher(Node):
    """Publish the measured camera optical frame once as a static TF."""

    def __init__(self):
        """Load the contract, publish the static transform and spin."""
        super().__init__('armpi_camera_frame_publisher')
        self.declare_parameter('config_file', default_config_path())
        config_file = str(self.get_parameter('config_file').value)
        self.config = load_frame_config(config_file)
        self.broadcaster = StaticTransformBroadcaster(self)
        message = transform_message(self.config)
        message.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(message)
        self.get_logger().info(
            'published static frame {} -> {} from {}'.format(
                self.config['parent_frame'],
                self.config['child_frame'],
                config_file,
            )
        )


def main(argv=None):
    """Run the static frame publisher."""
    rclpy.init(args=argv)
    node = CameraFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
