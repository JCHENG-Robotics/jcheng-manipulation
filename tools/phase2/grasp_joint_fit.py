#!/usr/bin/env python3
"""Fit the deep grasp family's j1/j4 to a measured top-face point.

All four 2026-11-19 floor grasps used the same family: ``joint2`` and
``joint3`` at their adopted low bounds, ``joint5 = 0``, ``joint1`` pointing at
the target azimuth and ``joint4`` setting the reach. This tool reproduces that
fit without MoveIt, so a paired trial does not depend on a running
``move_group``.

Run on the laptop with the Jazzy workspace sourced::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/grasp_joint_fit.py \
        --target 0.23992,-0.00205,-0.04166

The fit matches the recorded validation run to sub-millimetre x/y and returns
the same ``j1``/``j4`` to a few milliradians. ``z`` is not fitted: the family's
TCP height follows from ``j2``/``j3`` and sits 4-6 mm above the measured top
face, which the pads span. A target beyond the family's reach is reported with
its positive x residual rather than silently clamped.
"""

import argparse

import numpy as np

DEEP_J2 = -0.6618
DEEP_J3 = -1.8012
DEFAULT_J5 = 0.0
# Adopted joint4 range from the EEPROM-bounded mapping.
J4_LIMITS = (-1.5917, 1.5917)


class Kinematics:
    """Cached KDL forward kinematics for the nominal TCP."""

    def __init__(self):
        from armpi_grasp_planner.reachability import ArmKinematics

        self._tcp = ArmKinematics(tip_link='end_effector_link')

    def tcp(self, joints):
        """Return the TCP position for five joint values."""
        frame = self._tcp.forward([float(value) for value in joints])
        return np.array([frame.p.x(), frame.p.y(), frame.p.z()])


def _numerical_jacobian(function, joints, step=1e-6):
    """Return the 2x2 Jacobian of the TCP xy position over (j1, j4)."""
    base = function(joints)
    jacobian = np.empty((2, 2))
    for index, joint in enumerate((0, 3)):
        delta = list(joints)
        delta[joint] += step
        jacobian[:, index] = (function(delta) - base) / step
    return jacobian


def fit(target, kinematics=None, j2=DEEP_J2, j3=DEEP_J3, j5=DEFAULT_J5,
        max_iterations=50):
    """Fit j1/j4 so the TCP xy lands on *target*.

    Returns a dict with the five joints, the TCP position, the xy residual in
    millimetres and the solver status. The z residual is reported for context
    but is not minimised.
    """
    kinematics = kinematics or Kinematics()
    target = np.asarray(target, dtype=np.float64).reshape(3)
    joints = [0.0, float(j2), float(j3), -0.35, float(j5)]

    def tcp_xy(values):
        return kinematics.tcp(values)[:2]

    cost = float(np.linalg.norm(tcp_xy(joints) - target[:2]))
    damping = 1e-3
    for _ in range(max_iterations):
        residual = tcp_xy(joints) - target[:2]
        jacobian = _numerical_jacobian(tcp_xy, joints)
        normal = jacobian.T @ jacobian
        gradient = jacobian.T @ residual
        proposal = np.linalg.solve(
            normal + damping * (np.diag(np.diag(normal)) + 1e-12 * np.eye(2)),
            -gradient)
        candidate = list(joints)
        candidate[0] += float(proposal[0])
        candidate[3] = float(np.clip(
            candidate[3] + proposal[1], *J4_LIMITS))
        candidate_cost = float(np.linalg.norm(
            tcp_xy(candidate) - target[:2]))
        if candidate_cost < cost:
            improvement = cost - candidate_cost
            joints = candidate
            cost = candidate_cost
            damping = max(damping * 0.3, 1e-12)
            if improvement < 1e-12:
                break
        else:
            damping *= 10.0
            if damping > 1e12:
                break
    tcp = kinematics.tcp(joints)
    return {
        'joints': [float(value) for value in joints],
        'tcp': [float(value) for value in tcp],
        'target': [float(value) for value in target],
        'residual_mm': [float(value * 1000.0)
                        for value in (tcp - target)],
        'xy_error_mm': float(cost * 1000.0),
        'reach_limited': bool(
            joints[3] <= J4_LIMITS[0] + 1e-6 and cost > 0.002),
    }


def format_joints(joints, count=4):
    """Format the first *count* joints for a command line."""
    return ','.join('{:.4f}'.format(value) for value in joints[:count])


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(
        description='Fit the deep grasp family j1/j4 to a target point.')
    parser.add_argument('--target', required=True,
                        help='target x,y,z in base_link, comma separated')
    parser.add_argument('--j2', type=float, default=DEEP_J2)
    parser.add_argument('--j3', type=float, default=DEEP_J3)
    parser.add_argument('--j5', type=float, default=DEFAULT_J5)
    parser.add_argument('--json-out', help='write the fit as JSON')
    return parser


def main(argv=None):
    """Run the fit and print the command arguments."""
    args = build_parser().parse_args(argv)
    target = [float(value) for value in args.target.split(',')]
    if len(target) != 3:
        raise SystemExit('--target needs x,y,z')
    result = fit(target, j2=args.j2, j3=args.j3, j5=args.j5)
    joints = result['joints']
    print('joints (j1..j4): {}'.format(format_joints(joints)))
    print('TCP: ({:+.4f}, {:+.4f}, {:+.4f}) m'.format(*result['tcp']))
    print('residual: ({:+.2f}, {:+.2f}, {:+.2f}) mm  xy {:.2f} mm'.format(
        *result['residual_mm'], result['xy_error_mm']))
    if result['reach_limited']:
        print('note: joint4 reached its low limit; the target is beyond the '
              'family reach')
    if args.json_out:
        import json
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump(result, handle, indent=2)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
