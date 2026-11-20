#!/usr/bin/env python3
"""Publish the off-board front camera as a MoveIt collision object.

The D435i is bolted to the chassis front, ~20 mm below the link2 joint origin
at the low poses. It is not in the robot description (the live robot runs the
vendor URDF), so it is added to the planning scene the same way the measured
table is (`table_collision.py`): a box published to ``/collision_object`` and
republished so a MoveIt restart picks it up again.

The box is the camera body, placed with the calibrated ``base_link ->
ext_cam_optical`` transform (2026-11-20):

    python3 tools/phase2/ext_cam_collision.py \
        --extrinsic evidence/calibration/20261120/front_camera_extrinsic.json

The parent frame defaults to ``base_link`` because the camera is rigid to the
robot; a base move invalidates nothing for this object. ``--remove`` clears it.
This tool only touches the planning scene; it never moves the robot.
"""

import argparse
import json
import math

# RealSense D435i body: 90 x 25 x 25 mm.  In the optical frame x is width,
# y is height and z points forward out of the lens, so the body extends behind
# the lens origin.
BODY_SIZE = (0.090, 0.025, 0.026)
BODY_OFFSET = (0.0, 0.0, 0.010)
DEFAULT_OBJECT_ID = 'ext_cam_body'
DEFAULT_TOPIC = '/collision_object'
DEFAULT_FRAME = 'base_link'


def rotation_matrix_to_quaternion(rotation):
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion."""
    m = [[float(rotation[i][j]) for j in range(3)] for i in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x = 0.25 * s, (m[2][1] - m[1][2]) / s
        y, z = (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w, x = (m[2][1] - m[1][2]) / s, 0.25 * s
        y, z = (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w, x = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s
        y, z = 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w, x = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s
        y, z = (m[1][2] + m[2][1]) / s, 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def load_transform(path):
    """Return the 4x4 ``T_base_front`` from an extrinsic report."""
    with open(path) as handle:
        report = json.load(handle)
    return [[float(value) for value in row]
            for row in report['T_base_front']]


def compose_box(transform, size=BODY_SIZE, offset=BODY_OFFSET):
    """Place the camera box in the parent frame.

    Returns ``(center, quaternion, size)`` with the box's centre in the parent
    frame and its orientation from the transform.
    """
    rotation = [row[:3] for row in transform[:3]]
    center = [
        sum(rotation[i][j] * offset[j] for j in range(3)) + transform[i][3]
        for i in range(3)
    ]
    return center, rotation_matrix_to_quaternion(rotation), tuple(size)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish the front camera as a MoveIt collision object.')
    parser.add_argument('--extrinsic', required=True)
    parser.add_argument('--frame', default=DEFAULT_FRAME)
    parser.add_argument('--topic', default=DEFAULT_TOPIC)
    parser.add_argument('--object-id', default=DEFAULT_OBJECT_ID)
    parser.add_argument('--padding', type=float, default=0.0,
                        help='extra size on every axis (metres)')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=3.0)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--remove', action='store_true')
    args = parser.parse_args(argv)

    transform = load_transform(args.extrinsic)
    size = tuple(dimension + args.padding for dimension in BODY_SIZE)
    center, quaternion, size = compose_box(transform, size=size)

    import rclpy
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from rclpy.qos import (DurabilityPolicy, QoSProfile,
                           ReliabilityPolicy)
    from shape_msgs.msg import SolidPrimitive

    rclpy.init(args=[])
    node = rclpy.create_node('ext_cam_collision')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
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
        primitive.dimensions = list(size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = center
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = quaternion
        message.primitives.append(primitive)
        message.primitive_poses.append(pose)

    def publish():
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)

    publish()
    node.get_logger().info('{} {} at [{:.3f}, {:.3f}, {:.3f}] in {}'.format(
        'removed' if args.remove else 'published', args.object_id,
        center[0], center[1], center[2], args.frame))
    if args.once or args.rate <= 0.0:
        node.create_timer(0.5, publish)
        end = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    else:
        node.create_timer(1.0 / args.rate, publish)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
