#!/usr/bin/env python3
"""PROVISIONAL open-loop odometry for bench bring-up only.

This node integrates the commanded ``/cmd_vel`` and publishes ``/odom`` plus
the ``odom -> base_link`` transform. It has **no wheel encoder, no IMU and no
slip feedback**, so its pose drifts with every wheel slip, load change or
battery sag. It exists only to exercise the Nav2/TF graph before the LiDAR and
laser odometry are installed. Do not use its output for accuracy claims, long
runs, or any closed-loop test that matters.

The real odometry source for the AMR is ``rtabmap_odom/icp_odometry`` with the
LSLIDAR M10 scan; see ``config/icp_odometry.yaml``.

QoS note: ``/odom`` is published RELIABLE because Nav2's
``velocity_smoother``/``bt_navigator`` subscribe with the system default QoS.
The best-effort rule used for perception state streams does not apply to a
topic that Nav2 consumes.
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class OpenLoopOdom(Node):
    """Integrate /cmd_vel into a drifting odom pose (bench only)."""

    def __init__(self):
        super().__init__('open_loop_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('max_dt', 0.1)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.max_dt = float(self.get_parameter('max_dt').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)

        if self.rate_hz <= 0.0:
            raise ValueError('rate_hz must be > 0')
        if self.cmd_timeout < 0.0:
            raise ValueError('cmd_timeout must be >= 0')
        if self.max_dt <= 0.0:
            raise ValueError('max_dt must be > 0')

        # Pose and last command, in the odom frame / base frame.
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

        now = self.get_clock().now()
        self._last_tick = now
        self._last_cmd = now
        self._last_warn_ns = 0

        # RELIABLE: Nav2 subscribes to /odom with the system default QoS.
        odom_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._odom_pub = self.create_publisher(Odometry, 'odom', odom_qos)
        self._cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_cb, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().warn(
            'PROVISIONAL open-loop odometry: integrating /cmd_vel only, no '
            'wheel or IMU feedback. Bench use only; use rtabmap_odom/'
            'icp_odometry for the AMR.')

    def _cmd_cb(self, msg):
        self._vx = float(msg.linear.x)
        self._vy = float(msg.linear.y)
        self._wz = float(msg.angular.z)
        self._last_cmd = self.get_clock().now()

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        if dt <= 0.0:
            return
        if dt > self.max_dt:
            if now.nanoseconds - self._last_warn_ns > 5_000_000_000:
                self.get_logger().warn(
                    f'odom dt {dt:.3f} s exceeded max_dt {self.max_dt:.3f} s; '
                    'clamping (executor stall?)')
                self._last_warn_ns = now.nanoseconds
            dt = self.max_dt

        if (now - self._last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0

        # Integrate with the mid-interval yaw so constant arcs do not
        # systematically under/overshoot.
        yaw_mid = self._yaw + 0.5 * self._wz * dt
        vx = self._vx * self.linear_scale
        vy = self._vy * self.linear_scale
        self._x += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt
        self._y += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt
        self._yaw = wrap_angle(self._yaw + self._wz * self.angular_scale * dt)

        stamp = now.to_msg()
        quat = yaw_to_quaternion(self._yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        # Placeholder covariances: this model has no real uncertainty estimate.
        odom.pose.covariance[0] = 0.05 ** 2
        odom.pose.covariance[7] = 0.05 ** 2
        odom.pose.covariance[35] = 0.10 ** 2
        odom.twist.covariance[0] = 0.05 ** 2
        odom.twist.covariance[7] = 0.05 ** 2
        odom.twist.covariance[35] = 0.10 ** 2
        self._odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = quat
            self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdom()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
