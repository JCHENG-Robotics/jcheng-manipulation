#!/usr/bin/env python3
"""Vendor bus-servo mapping and limits for the ArmPi Ultra arm.

Single source for the pulse<->radian convention and for the guard rails the
fast-motion and grasp tools share. Values are the measured ones:

* 1000 pulses = 240 deg, so one pulse is 0.0041888 rad (vendor
  ``joint_position_controller.py``);
* ``joint1..joint5`` are servo IDs 6,5,4,3,2 and ``joint3`` is flipped;
* hard bounds are the servo EEPROM position limits, tightened by the measured
  mechanical stops where those are lower (joint2 low 322 from the 2026-11-19
  hand survey);
* adopted bounds are the EEPROM limit minus the 20-pulse margin used by the
  URDF and the MoveIt ``joint_limits.yaml`` (safety envelope, not a mechanic
  stop -- the validated view pose sits below the adopted joint4 minimum).

Pure Python (no ROS) so the laptop can unit-test it.
"""

import math

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
SERVO_IDS = {'gripper': 1, 'joint1': 6, 'joint2': 5, 'joint3': 4,
             'joint4': 3, 'joint5': 2}
JOINT_ORDER = [6, 5, 4, 3, 2]
FLIPPED = {'joint1': False, 'joint2': False, 'joint3': True,
           'joint4': False, 'joint5': False}
RAD_PER_TICK = 240.0 / 360.0 * (2.0 * math.pi) / 1000.0
TICKS_PER_RAD = 1.0 / RAD_PER_TICK

HARD_BOUNDS = {'joint1': (50, 950), 'joint2': (322, 830),
               'joint3': (500, 950), 'joint4': (100, 900),
               'joint5': (100, 890)}
ADOPTED_BOUNDS = {'joint1': (70, 930), 'joint2': (342, 810),
                  'joint3': (500, 930), 'joint4': (120, 880),
                  'joint5': (120, 870)}

GRIPPER_OPEN_PULSE = 170
GRIPPER_CLOSED_PULSE = 700
#: Measured grip values for the 39.4 mm cube in ``r_joint`` radians.
GRIPPER_OPEN_RAD = -2.22
GRIPPER_PRECLOSE_RAD = -1.6
GRIPPER_GRIP_RAD = -0.63

#: A commanded joint angle maps to a register target through the vendor's
#: ``init`` (500 pulse), but the realised register sits up to ~9 pulses away
#: (measured: joint4 at the view pose is commanded to 93 and reads 101-103).
#: The bounds check therefore allows this margin so a validated pose is not
#: rejected as a typo; anything further out is refused.
COMMAND_MARGIN_PULSES = 12


def pulse_to_rad(joint, pulse):
    """Convert a measured register to the vendor commanded joint angle."""
    offset = (500.0 - pulse) if FLIPPED[joint] else (pulse - 500.0)
    return offset * RAD_PER_TICK


def rad_to_pulse(joint, angle):
    """Convert a commanded joint angle to the vendor register target."""
    offset = angle * TICKS_PER_RAD
    return int(round(500.0 - offset if FLIPPED[joint] else 500.0 + offset))


def gripper_pulse(angle_rad):
    """Convert ``r_joint`` radians to the raw gripper servo pulse."""
    return int(round(700.0 + angle_rad * TICKS_PER_RAD))


def waypoint_pulses(waypoints):
    """Return ``{joint_name: [pulse, ...]}`` for a list of joint vectors."""
    return {name: [rad_to_pulse(name, point[index])
                   for point in waypoints]
            for index, name in enumerate(ARM_JOINTS)}


def check_waypoints(waypoints):
    """Refuse a waypoint outside the servo limits; warn outside adopted."""
    errors = []
    warnings = []
    margin = COMMAND_MARGIN_PULSES
    for name, pulses in waypoint_pulses(waypoints).items():
        low, high = HARD_BOUNDS[name]
        if min(pulses) < low - margin or max(pulses) > high + margin:
            errors.append('{} pulse [{}, {}] outside hard bounds {}'.format(
                name, min(pulses), max(pulses), (low, high)))
        low, high = ADOPTED_BOUNDS[name]
        if min(pulses) < low - margin or max(pulses) > high + margin:
            warnings.append('{} pulse [{}, {}] outside adopted {}'.format(
                name, min(pulses), max(pulses), (low, high)))
    if errors:
        raise ValueError('; '.join(errors))
    return warnings
