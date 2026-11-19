#!/usr/bin/env python3
"""Drive the mecanum base along one bounded ground-plane direction.

The direction is a unit vector in the base frame: ``--vx`` forward, ``--vy``
left. Each wheel gets

    duty * (vx * SIGN_X[id] + vy * SIGN_Y[id])

with the operator-confirmed 2026-11-18 sign tables (forward ``[+,−,−,+]``,
left ``[+,+,+,+]`` for IDs 1–4 = RF, RR, LF, LR). The node publishes
``MotorsSpeedControl`` on the vendor topic at 20 Hz for one pulse and then an
explicit zero.

This is a supervised, open-loop tool: there is no odometry, and any closed-loop
placement must re-measure the scene between pulses. A pulse shorter than
~0.15 s stays inside the dead band. The default duration cap is 3 s; the
6.3 s straight-line return used in the 2026-11-19 out-of-view run needs
``--allow-long-pulse`` (cap 7 s).

Run inside the vendor container, with the vendor CycloneDDS environment:

    source ~/.zshrc >/dev/null 2>&1
    python3 base_pulse_vector.py --vx 0.414 --vy 0.910 --duty 10 \\
        --duration 2.8 --yes
"""

import argparse
import json
import math
import time

TOPIC = '/ros_robot_controller/set_motor_speeds'
PUBLISH_HZ = 20.0
MAX_WHEEL_DUTY = 15.0
DEFAULT_MAX_DURATION = 3.0
LONG_MAX_DURATION = 7.0
# operator-confirmed 2026-11-18, IDs 1..4 = RF, RR, LF, LR
SIGN_X = {1: 1.0, 2: -1.0, 3: -1.0, 4: 1.0}
SIGN_Y = {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0}


def wheel_speeds(vx, vy, duty):
    """Return ``{motor_id: duty}`` for a base-frame unit direction.

    The direction is normalised first, so the caller can pass any vector.
    """
    norm = math.hypot(vx, vy)
    if norm == 0.0:
        raise ValueError('zero direction')
    vx, vy = vx / norm, vy / norm
    return {mid: duty * (vx * SIGN_X[mid] + vy * SIGN_Y[mid])
            for mid in (1, 2, 3, 4)}


def main():
    """Publish one bounded pulse, then an explicit zero."""
    import rclpy
    from rclpy.node import Node
    from ros_robot_controller_msgs.msg import (
        MotorSpeedControl,
        MotorsSpeedControl,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vx', type=float, required=True,
                        help='forward component of the unit direction')
    parser.add_argument('--vy', type=float, required=True,
                        help='left component of the unit direction')
    parser.add_argument('--duty', type=float, default=10.0)
    parser.add_argument('--duration', type=float, default=1.0)
    parser.add_argument(
        '--allow-long-pulse', action='store_true',
        help='raise the duration cap from 3 s to 7 s (supervised)')
    parser.add_argument('--yes', action='store_true')
    args = parser.parse_args()
    if not args.yes:
        raise SystemExit('pass --yes to move the base')
    norm = math.hypot(args.vx, args.vy)
    if norm == 0.0:
        raise SystemExit('zero direction')
    vx, vy = args.vx / norm, args.vy / norm
    if not 0 < args.duty <= 12.0:
        raise SystemExit('duty outside 0..12 %')
    max_duration = (
        LONG_MAX_DURATION if args.allow_long_pulse else DEFAULT_MAX_DURATION)
    if not 0 < args.duration <= max_duration:
        raise SystemExit(
            'duration outside 0..{} s'.format(max_duration))
    wheels = wheel_speeds(args.vx, args.vy, args.duty)
    if max(abs(v) for v in wheels.values()) > MAX_WHEEL_DUTY:
        raise SystemExit('wheel duty above {} %'.format(MAX_WHEEL_DUTY))

    rclpy.init()
    node = Node('base_pulse_vector')
    publisher = node.create_publisher(MotorsSpeedControl, TOPIC, 10)
    time.sleep(1.5)
    message = MotorsSpeedControl()
    for motor_id in (1, 2, 3, 4):
        message.data.append(MotorSpeedControl(
            id=motor_id, speed=float(wheels[motor_id])))
    period = 1.0 / PUBLISH_HZ
    start = time.monotonic()
    sent = 0
    while time.monotonic() - start < args.duration:
        publisher.publish(message)
        sent += 1
        time.sleep(period)
    zero = MotorsSpeedControl()
    for motor_id in (1, 2, 3, 4):
        zero.data.append(MotorSpeedControl(id=motor_id, speed=0.0))
    for _ in range(10):
        publisher.publish(zero)
        sent += 1
        time.sleep(period)
    print(json.dumps({
        'vx': vx, 'vy': vy, 'duty': args.duty,
        'duration': args.duration, 'wheel_speeds': wheels,
        'messages': sent, 'zero_written': True,
    }))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
