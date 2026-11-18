#!/usr/bin/env python3
"""
Publish the measured table plane as a MoveIt collision object.

The safety layer checks the TCP against the table plane, but it cannot see the
arm or gripper geometry. This tool adds the table as a box in the planning
scene so MoveIt's collision checker also protects link3-5 and the fingers.

The box is oriented with its **top face on the measured plane** and extends
0.05 m below it. It is republished at 1 Hz so a MoveIt restart picks it up
again; `--remove` clears it. Run this before the grasp search and keep it
alive for the execute path:

    python3 tools/phase2/table_collision.py \
        --plane-json \
        evidence/phase2/onsite_20261118/depth_plane_optical_20261118.json

The plane file uses `plane_base.normal` and `plane_base.offset` with the
convention `normal . p + offset = 0` (the same convention as
`armpi_safety/config/safety_limits.yaml`).

This tool only touches the planning scene; it never moves the robot.
"""

import argparse
import json
import math

DEFAULT_NORMAL = (
    0.010305373291790987,
    -0.06301369005824492,
    0.9979594551616625,
)
DEFAULT_OFFSET = 0.07728284112861183
DEFAULT_OBJECT_ID = 'table_surface'
DEFAULT_SIZE_X = 1.0
DEFAULT_SIZE_Y = 1.0
DEFAULT_THICKNESS = 0.05
EPS = 1e-12


def normalize(vector):
    """Return a unit vector, raising on a zero vector."""
    length = math.sqrt(sum(value * value for value in vector))
    if length <= EPS:
        raise ValueError('cannot normalize a zero vector')
    return tuple(float(value) / length for value in vector)


def load_plane(path):
    """Read ``(normal, offset)`` from a depth-plane report."""
    with open(path, 'r', encoding='utf-8') as handle:
        report = json.load(handle)
    for key in ('plane_base', 'plane', 'table_plane'):
        if key in report:
            plane = report[key]
            return (
                tuple(float(value) for value in plane['normal']),
                float(plane['offset']),
            )
    raise ValueError(
        '{} has no plane_base/plane/table_plane section'.format(path)
    )


def quaternion_from_z_axis(normal):
    """Return the quaternion that rotates +z onto *normal*."""
    n = normalize(normal)
    if n[2] <= -1.0 + 1e-9:
        raise ValueError('table plane normal must point up (+z)')
    horizontal = math.sqrt(n[0] * n[0] + n[1] * n[1])
    if horizontal <= EPS:
        return (0.0, 0.0, 0.0, 1.0)
    axis = (-n[1] / horizontal, n[0] / horizontal, 0.0)
    angle = math.acos(max(-1.0, min(1.0, n[2])))
    sine = math.sin(angle / 2.0)
    return (
        axis[0] * sine,
        axis[1] * sine,
        axis[2] * sine,
        math.cos(angle / 2.0),
    )


def rotate_vector(quaternion, vector):
    """Rotate *vector* by a unit quaternion (x, y, z, w)."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def table_box(normal, offset, size_x, size_y, thickness):
    """
    Return ``(center, quaternion, dimensions)`` for the table box.

    The plane is ``normal . p + offset = 0`` and the normal points up. The
    box's +z axis is aligned with the normal and its top face lies on the
    plane, so the box occupies the space below the table surface.
    """
    n = normalize(normal)
    if n[2] <= 0.0:
        raise ValueError('table plane normal must point up (+z)')
    if size_x <= 0.0 or size_y <= 0.0 or thickness <= 0.0:
        raise ValueError('table box dimensions must be positive')
    # Closest point on the plane to the origin, then drop by half thickness.
    surface = tuple(-offset * value for value in n)
    center = tuple(
        surface[index] - n[index] * thickness / 2.0
        for index in range(3)
    )
    quaternion = quaternion_from_z_axis(n)
    return center, quaternion, (size_x, size_y, thickness)


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Publish the measured table as a MoveIt collision object.'
    )
    parser.add_argument(
        '--plane-json',
        default=None,
        help='depth-plane report; overrides --normal/--offset',
    )
    parser.add_argument(
        '--normal',
        nargs=3,
        type=float,
        default=list(DEFAULT_NORMAL),
        metavar=('NX', 'NY', 'NZ'),
        help='table plane normal (must point up)',
    )
    parser.add_argument(
        '--offset',
        type=float,
        default=DEFAULT_OFFSET,
        help='plane offset: normal . p + offset = 0',
    )
    parser.add_argument('--frame', default='base_link')
    parser.add_argument('--object-id', default=DEFAULT_OBJECT_ID)
    parser.add_argument(
        '--topic', default='/collision_object', help='MoveIt scene topic'
    )
    parser.add_argument('--size-x', type=float, default=DEFAULT_SIZE_X)
    parser.add_argument('--size-y', type=float, default=DEFAULT_SIZE_Y)
    parser.add_argument(
        '--thickness', type=float, default=DEFAULT_THICKNESS
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=1.0,
        help='republish rate in Hz; 0 with --once publishes a single message',
    )
    parser.add_argument(
        '--once',
        action='store_true',
        help='publish at 2 Hz for --timeout, then exit',
    )
    parser.add_argument(
        '--remove',
        action='store_true',
        help='remove the table object instead of adding it',
    )
    parser.add_argument('--timeout', type=float, default=5.0)
    return parser.parse_args(argv)


def main(argv=None):
    """Run the publisher. Returns a process exit code."""
    args = _parse_args(argv)
    if args.plane_json:
        normal, offset = load_plane(args.plane_json)
    else:
        normal, offset = tuple(args.normal), args.offset
    center, quaternion, dimensions = table_box(
        normal, offset, args.size_x, args.size_y, args.thickness
    )

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from shape_msgs.msg import SolidPrimitive

    rclpy.init(args=[])
    node = rclpy.create_node('table_collision')
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    publisher = node.create_publisher(CollisionObject, args.topic, qos)
    message = CollisionObject()
    message.header.frame_id = args.frame
    message.id = args.object_id
    if args.remove:
        message.operation = CollisionObject.REMOVE
    else:
        message.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(dimensions)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = quaternion
        message.primitives.append(primitive)
        message.primitive_poses.append(pose)

    def publish():
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)

    publish()
    if args.once or args.rate <= 0.0:
        # Repeat during the bounded window so one lost message cannot leave the
        # scene unchanged (the first --once design published only once).
        node.create_timer(0.5, publish)
        end = node.get_clock().now().nanoseconds + int(
            args.timeout * 1e9
        )
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        node.create_timer(1.0 / args.rate, publish)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
