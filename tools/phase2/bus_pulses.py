#!/usr/bin/env python3
"""Read the five arm bus-servo registers and print them as CSV.

Read-only: it never commands a servo. Run inside the robot container with the
vendor environment (CycloneDDS)::

    python3 /tmp/bus_pulses.py
    # 496,555,792,103,496

The order is ``joint1..joint5`` = servo IDs 6,5,4,3,2 (vendor
``servo_controller.yaml``). The output is what
``object_segment_rgbd.py --joints-pulses`` and the measured-joint FK expect.

A register at a negative position crashes ``ros_robot_controller`` on this
firmware (uint16 assertion), so this tool refuses to query an ID whose last
known position is negative and reports the read failure instead of retrying.
It cannot know the value before reading, so it keeps the query set to the five
arm joints, which sit above zero at every pose used by the project.
"""

import argparse
import json
import sys

import rclpy
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState
from rclpy.node import Node

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
SERVO_IDS = {'joint1': 6, 'joint2': 5, 'joint3': 4, 'joint4': 3, 'joint5': 2}
RADIANS_PER_TICK = 240.0 / 360.0 * (2.0 * 3.141592653589793) / 1000.0
JOINT_SIGN = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0,
              'joint4': 1.0, 'joint5': 1.0}


def pulses_to_radians(pulses):
    """Convert five register values to measured joint angles."""
    return [JOINT_SIGN[name] * (float(pulse) - 500.0) * RADIANS_PER_TICK
            for name, pulse in zip(ARM_JOINTS, pulses)]


def read_pulses(timeout=5.0):
    """Read the five arm registers; return a list in joint order."""
    request = GetBusServoState.Request()
    for name in ARM_JOINTS:
        command = GetBusServoCmd()
        command.id = int(SERVO_IDS[name])
        command.get_position = 1
        request.cmd.append(command)
    node = Node('bus_pulses')
    try:
        client = node.create_client(
            GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('servo service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('servo read failed')
        positions = {}
        for command, state in zip(request.cmd, response.state):
            if state.position:
                positions[int(command.id)] = int(state.position[0])
        missing = [name for name in ARM_JOINTS
                   if SERVO_IDS[name] not in positions]
        if missing:
            raise RuntimeError('no position for {}'.format(missing))
        return [positions[SERVO_IDS[name]] for name in ARM_JOINTS]
    finally:
        node.destroy_node()


def main(argv=None):
    """Read and print the registers."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json-out', default=None)
    args = parser.parse_args(argv)
    rclpy.init()
    try:
        pulses = read_pulses()
    except Exception as error:  # noqa: BLE001 - report and fail loudly
        print('pulse read failed: {}'.format(error), file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()
    print(','.join(str(value) for value in pulses))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump({
                'pulses': pulses,
                'joints': pulses_to_radians(pulses),
                'servo_ids': [SERVO_IDS[name] for name in ARM_JOINTS],
            }, handle, indent=2)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
