"""Re-engage bus-servo torque by holding the current positions.

The vendor bus servos do not reliably re-enable torque from the
``SetBusServoState`` 0x0B command alone after a torque-off. A position command
at the current register values re-engages the servo and holds the pose.

The tool reads the current positions, publishes them as a slow
``ServosPosition`` command (default 1.0 s), and repeats the hold. It is a
recovery tool: the arm must already be supported, and the caller should check
that the current pose is safe before releasing support.

Run inside the vendor container:

    python3 servo_hold.py --duration 1.0 --repeats 3
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import (
    GetBusServoCmd,
    ServoPosition,
    ServosPosition,
)
from ros_robot_controller_msgs.srv import GetBusServoState

SERVO_IDS = [1, 2, 3, 4, 5, 6]
STATE_SERVICE = '/ros_robot_controller/bus_servo/get_state'
POSITION_TOPIC = '/ros_robot_controller/bus_servo/set_position'


class HoldNode(Node):
    """Read positions and publish a hold command."""

    def __init__(self):
        super().__init__('servo_hold')
        self.client = self.create_client(GetBusServoState, STATE_SERVICE)
        self.publisher = self.create_publisher(
            ServosPosition, POSITION_TOPIC, 10)

    def read_positions(self, get_torque=False):
        """Return ``{servo_id: position}`` and optionally torque states."""
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('bus servo state service not available')
        request = GetBusServoState.Request()
        for servo_id in SERVO_IDS:
            command = GetBusServoCmd()
            command.id = servo_id
            command.get_position = 1
            command.get_torque_state = 1 if get_torque else 0
            request.cmd.append(command)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('bus servo state read failed')
        positions = {}
        torques = {}
        for command, state in zip(request.cmd, response.state):
            positions[int(command.id)] = (
                int(state.position[0]) if state.position else None)
            torques[int(command.id)] = (
                int(state.enable_torque[0]) if state.enable_torque else None)
        return positions, torques

    def hold(self, positions, duration):
        """Publish one hold command at the supplied positions."""
        message = ServosPosition()
        message.duration = float(duration)
        for servo_id in SERVO_IDS:
            entry = ServoPosition()
            entry.id = servo_id
            entry.position = int(positions[servo_id])
            message.position.append(entry)
        self.publisher.publish(message)
        self.get_logger().info(
            'hold command: ' + ', '.join(
                f'{servo_id}={positions[servo_id]}'
                for servo_id in SERVO_IDS))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--interval', type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = HoldNode()
    try:
        positions, torques = node.read_positions(get_torque=True)
        print(f'before positions: {positions}')
        print(f'before torque: {torques}')
        for index in range(max(1, args.repeats)):
            node.hold(positions, args.duration)
            time.sleep(args.duration + args.interval)
            print(f'hold {index + 1}/{args.repeats} sent')
        after_positions, after_torques = node.read_positions(get_torque=True)
        print(f'after positions: {after_positions}')
        print(f'after torque: {after_torques}')
        moved = {
            servo_id: after_positions[servo_id] - positions[servo_id]
            for servo_id in SERVO_IDS
            if after_positions[servo_id] is not None
        }
        print(f'position change (pulses): {moved}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
