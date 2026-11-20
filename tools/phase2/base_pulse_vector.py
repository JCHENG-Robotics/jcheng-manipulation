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

The speed is commanded in **duty** or in **mm/s** (``--speed-mm-s``) using the
2026-11-18 scale ``duty = 0.182 * v[mm/s]`` (10 % = 55 mm/s, 20 % = 110 mm/s).
The vector duty is capped at 20 %; a 45-degree diagonal commands
``20 * sqrt(2) = 28.3`` on two wheels, so the wheel cap is 30. The publisher
waits for a matched subscriber (the vendor watchdog drops a command with no
consumer) instead of a fixed discovery sleep.

Run inside the vendor container, with the vendor CycloneDDS environment:

    source ~/.zshrc >/dev/null 2>&1
    python3 base_pulse_vector.py --vx 0.414 --vy 0.910 --speed-mm-s 110 \\
        --duration 1.4 --yes
"""

import argparse
import json
import math
import sys
import time

TOPIC = '/ros_robot_controller/set_motor_speeds'
PUBLISH_HZ = 20.0
MAX_DUTY = 20.0
MAX_WHEEL_DUTY = 30.0
DUTY_PER_MM_S = 0.182
#: 20 % duty measured 110 mm/s; 110 * 0.182 = 20.02, so the duty is clamped.
MAX_SPEED_MM_S = 110.0
DEFAULT_MAX_DURATION = 3.0
LONG_MAX_DURATION = 7.0
ZERO_MESSAGES = 4
DISCOVERY_TIMEOUT_S = 3.0
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


def speed_to_duty(speed_mm_s):
    """Convert a vector speed in mm/s to duty with the measured scale."""
    speed_mm_s = abs(float(speed_mm_s))
    if speed_mm_s > MAX_SPEED_MM_S + 1e-9:
        raise ValueError('speed {:.1f} mm/s above the {:.1f} mm/s cap'.format(
            speed_mm_s, MAX_SPEED_MM_S))
    return min(speed_mm_s * DUTY_PER_MM_S, MAX_DUTY)


def travel_mm(duty, duration):
    """Return the nominal translation (mm) of a pulse."""
    return float(duty) * float(duration) / DUTY_PER_MM_S


def check_duty(duty, wheels):
    """Refuse a duty outside the verified envelope."""
    if not 0.0 < duty <= MAX_DUTY:
        raise ValueError('duty {:.1f} outside 0..{:.0f} %'.format(
            duty, MAX_DUTY))
    peak = max(abs(value) for value in wheels.values())
    if peak > MAX_WHEEL_DUTY:
        raise ValueError(
            'wheel duty {:.1f} % above {:.0f} %; use a lower duty or an '
            'axis-aligned direction'.format(peak, MAX_WHEEL_DUTY))


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
    speed = parser.add_mutually_exclusive_group()
    speed.add_argument('--duty', type=float, default=10.0,
                       help='vector duty in percent (default 10)')
    speed.add_argument('--speed-mm-s', type=float,
                       help='vector speed in mm/s (duty = 0.182 * v)')
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
    if args.speed_mm_s is not None:
        try:
            duty = speed_to_duty(args.speed_mm_s)
        except ValueError as error:
            raise SystemExit(str(error)) from error
    else:
        duty = args.duty
    max_duration = (
        LONG_MAX_DURATION if args.allow_long_pulse else DEFAULT_MAX_DURATION)
    if not 0 < args.duration <= max_duration:
        raise SystemExit(
            'duration outside 0..{} s'.format(max_duration))
    wheels = wheel_speeds(args.vx, args.vy, duty)
    try:
        check_duty(duty, wheels)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    rclpy.init()
    node = Node('base_pulse_vector')
    publisher = node.create_publisher(MotorsSpeedControl, TOPIC, 10)
    # DDS discovery is a state, not a duration: wait for the vendor node to
    # match, then move. The old code slept 1.5 s here unconditionally.
    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    while (publisher.get_subscription_count() < 1
           and time.monotonic() < deadline):
        rclpy.spin_once(node, timeout_sec=0.02)
    matched = publisher.get_subscription_count()
    if matched < 1:
        print('WARNING: no subscriber matched on {} after {:.1f} s'.format(
            TOPIC, DISCOVERY_TIMEOUT_S), file=sys.stderr)
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
    for _ in range(ZERO_MESSAGES):
        publisher.publish(zero)
        sent += 1
        time.sleep(period)
    print(json.dumps({
        'vx': vx, 'vy': vy, 'duty': duty,
        'speed_mm_s': duty / DUTY_PER_MM_S,
        'duration': args.duration,
        'nominal_travel_mm': round(travel_mm(duty, args.duration), 1),
        'wheel_speeds': wheels, 'messages': sent,
        'subscribers_matched': matched, 'zero_written': True,
    }))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
