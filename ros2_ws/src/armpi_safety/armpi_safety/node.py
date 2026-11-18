"""
ROS 2 safety supervisor for ArmPi planned trajectories.

The node is a veto layer: it watches ``/joint_states``, publishes a status and
a latched stop flag, and exposes ``validate_trajectory``. It never executes a
trajectory and never publishes a controller command.
"""

import json

from arm_interfaces.srv import ValidateTrajectory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .policy import ARM_JOINTS, load_config, SafetyMonitor


def _default_config_path():
    """Return the installed safety configuration path."""
    from ament_index_python.packages import get_package_share_directory

    return (
        get_package_share_directory('armpi_safety')
        + '/config/safety_limits.yaml'
    )


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def trajectory_to_dict(message):
    """Convert a ``moveit_msgs/RobotTrajectory`` into the policy schema."""
    trajectory = message.joint_trajectory
    names = list(trajectory.joint_names)
    points = []
    for point in trajectory.points:
        points.append(
            {
                'positions': [float(value) for value in point.positions],
                'velocities': [float(value) for value in point.velocities],
                'accelerations': [
                    float(value) for value in point.accelerations
                ],
                'time_from_start_s': _stamp_seconds(
                    point.time_from_start
                ),
            }
        )
    return {'joint_names': names, 'points': points}


class SafetySupervisor(Node):
    """Publish watchdog state and validate planned trajectories."""

    def __init__(self):
        """Create the watchdog, publishers and services."""
        super().__init__('armpi_safety')
        self.declare_parameter(
            'config_file', _default_config_path()
        )
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('workspace_tip_link', 'end_effector_link')

        config_path = str(self.get_parameter('config_file').value)
        self.config = load_config(config_path)
        self.monitor = SafetyMonitor(self.config)
        self.kinematics = None
        self._kinematics_error = None
        self.workspace_tip_link = str(
            self.get_parameter('workspace_tip_link').value
        )

        stop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.stop_publisher = self.create_publisher(
            Bool, 'safety_stop', stop_qos
        )
        self.status_publisher = self.create_publisher(
            String, 'safety_status', 10
        )
        self.joint_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('joint_state_topic').value),
            self.on_joint_state,
            10,
        )
        self.validate_service = self.create_service(
            ValidateTrajectory,
            '~/validate_trajectory',
            self.on_validate_trajectory,
        )
        self.reset_service = self.create_service(
            Trigger, '~/reset_stop', self.on_reset_stop
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.publish_status)
        self.publish_status()
        self.get_logger().info(
            'safety supervisor: config={} state_timeout={:.3f}s '
            'workspace={} table={}'.format(
                config_path,
                self.config.state_timeout_s,
                'on' if self.config.workspace_enabled else 'off',
                'on' if self.config.table_plane_enabled else 'off',
            )
        )

    def on_joint_state(self, message):
        """Update the watchdog from one joint-state message."""
        self.monitor.update_joint_state(
            message.name, message.position, received_at=self._now()
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_kinematics(self):
        if self.kinematics is not None:
            return self.kinematics
        if self._kinematics_error is not None:
            return None
        try:
            from armpi_grasp_planner.reachability import ArmKinematics

            self.kinematics = ArmKinematics(tip_link=self.workspace_tip_link)
            self.get_logger().info(
                'workspace FK ready: base_link -> {}'.format(
                    self.workspace_tip_link
                )
            )
        except Exception as exc:  # noqa: BLE001 - report once, veto if needed
            self._kinematics_error = str(exc)
            self.get_logger().error(
                'workspace FK is unavailable: {}'.format(exc)
            )
            return None
        return self.kinematics

    def _tcp_fk(self, positions):
        kinematics = self._load_kinematics()
        if kinematics is None:
            return None
        joints = [float(positions[joint]) for joint in ARM_JOINTS]
        frame = kinematics.forward(joints)
        return (frame.p.x(), frame.p.y(), frame.p.z())

    def on_validate_trajectory(self, request, response):
        """Validate a trajectory and return a veto or approval."""
        trajectory = trajectory_to_dict(request.trajectory)
        report = self.monitor.validate_trajectory(
            trajectory,
            now=self._now(),
            tcp_fk=self._tcp_fk,
            max_start_state_distance_rad=(
                request.max_start_state_distance_rad
            ),
        )
        response.approved = bool(report['approved'])
        response.code = str(report['code'])
        response.violations = list(report['messages'])
        response.report_json = json.dumps(report)
        if report['approved']:
            self.get_logger().info(
                'trajectory approved: {} points, {:.3f}s'.format(
                    report['point_count'], report['duration_s']
                )
            )
        else:
            self.get_logger().warn(
                'trajectory rejected ({}): {}'.format(
                    report['code'], '; '.join(report['messages'])
                )
            )
        self.publish_status()
        return response

    def on_reset_stop(self, request, response):
        """Clear a latched stop when the current state is valid."""
        del request
        result = self.monitor.reset_stop(now=self._now())
        response.success = bool(result['ok'])
        response.message = str(result['message'])
        self.publish_status()
        return response

    def publish_status(self):
        """Publish the watchdog status and the latched stop flag."""
        status = self.monitor.status(now=self._now())
        message = String()
        message.data = json.dumps(status)
        self.status_publisher.publish(message)
        stop = Bool()
        stop.data = bool(status['stop'])
        self.stop_publisher.publish(stop)


def main(argv=None):
    """Run the safety supervisor."""
    rclpy.init(args=argv)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
