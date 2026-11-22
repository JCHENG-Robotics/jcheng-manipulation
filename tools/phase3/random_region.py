#!/usr/bin/env python3
"""Define the free-placement reference envelope of the 24-pair trial.

The 2026-11-21 follow-up does **not** pre-place 24 marks and does not ask the
operator to measure coordinates. The operator puts the same cube at a new
place, by eye, anywhere in the forward workspace before every pair; the robot
measures the realized point and records it. This tool writes the reference
envelope, the free-placement rule and the seeded pair/method plan (fixtures
``r01`` ... ``r24``) before data.

The reference envelope is the robot's **sensor-covered fan from the home
pose**, not a grid:

- radius about 0.25-0.55 m;
- bearing about -20 ... +25 deg from the base forward axis (+y is left);
- a new spot every pair (about 30 mm from the previous point is a guide).

The near part (radius <= ~0.46 m) is visible to the view-pose wrist camera or
a joint1 yaw sweep. The far part (0.46-0.55 m) is outside the wrist image; it
is covered by the chassis D435i, whose calibrated far fan is the reason for
the asymmetric bearing bounds. The envelope is a **guide**, not a gate: the
operator does not measure, the robot reports the realized point, and a point
too far or too wide to locate is simply moved a little closer by the operator.

Run on the laptop (no robot)::

    python3 tools/phase3/random_region.py \
        --output evidence/phase3/onsite_20261121_random/region.json \
        --plan evidence/phase3/onsite_20261121_random/paired_trials_plan.json
"""

import argparse
from datetime import datetime, timezone
import json
import os

SCHEMA_VERSION = 1
DEFAULT_N_PAIRS = 24
DEFAULT_SEED = 17
DEFAULT_MIN_RADIUS_M = 0.25
DEFAULT_MAX_RADIUS_M = 0.55
DEFAULT_MIN_BEARING_DEG = -20.0
DEFAULT_MAX_BEARING_DEG = 25.0
DEFAULT_MIN_SEPARATION_M = 0.03
FIXTURE_PREFIX = 'r'

from _bootstrap import bootstrap  # noqa: E402
bootstrap()


def fixture_id(index):
    """Return the pair fixture label for a zero-based index."""
    return '{}{:02d}'.format(FIXTURE_PREFIX, index + 1)


def build_region(n_pairs=DEFAULT_N_PAIRS, seed=DEFAULT_SEED,
                 min_radius=DEFAULT_MIN_RADIUS_M,
                 max_radius=DEFAULT_MAX_RADIUS_M,
                 min_bearing=DEFAULT_MIN_BEARING_DEG,
                 max_bearing=DEFAULT_MAX_BEARING_DEG,
                 min_separation=DEFAULT_MIN_SEPARATION_M):
    """Return the validated region payload."""
    if int(n_pairs) < 1:
        raise ValueError('n_pairs must be positive')
    if not 0.15 <= min_radius < max_radius <= 0.65:
        raise ValueError('radii must satisfy 0.15 <= min < max <= 0.65 m')
    if not -45.0 <= min_bearing < max_bearing <= 45.0:
        raise ValueError(
            'bearings must satisfy -45 <= min < max <= 45 deg')
    if not 0.0 < min_separation <= 0.10:
        raise ValueError('min_separation must be in (0, 0.10] m')
    return {
        'schema_version': SCHEMA_VERSION,
        'created_at': datetime.now(timezone.utc).isoformat(),
        'n_pairs': int(n_pairs),
        'seed': int(seed),
        'fixture_prefix': FIXTURE_PREFIX,
        'fixtures': [fixture_id(index) for index in range(int(n_pairs))],
        'region': {
            'min_radius_m': float(min_radius),
            'max_radius_m': float(max_radius),
            'min_bearing_deg': float(min_bearing),
            'max_bearing_deg': float(max_bearing),
        },
        'placement_rule': {
            'operator_random': True,
            'measured_by_operator': False,
            'min_separation_m': float(min_separation),
            'instructions': (
                'Before every pair the operator places the cube by eye at a '
                'new spot in the forward workspace; no marks and no '
                'measurements. Do not reuse the same spot. The robot reports '
                'the realized point and decides: if the point is already in '
                'the 0.226-0.234 m band it grasps directly, otherwise it '
                'drives the base toward it and then grasps. A point too far '
                'or too wide to locate is moved a little closer by the '
                'operator. The measured point, not the operator intent, is '
                'the recorded site.'),
        },
        'locators': ['wrist_view', 'front_camera', 'wrist_yaw'],
        'coverage': {
            'wrist_view_max_radius_m': 0.46,
            'front_camera': {
                'role': 'far locator (0.46-0.55 m) and fallback',
                'note': (
                    'The far bearing bounds follow the calibrated D435i '
                    'far-field fan from the home pose. Its depth top-face '
                    'cluster is used only to drive the base into the wrist '
                    'band; the final grasp target is always the validated '
                    'wrist measurement.'),
            },
            'note': (
                'The radius/bearing bounds are a guide to where the robot '
                'can see the cube, not a gate. The operator does not '
                'measure; the realized point is recorded and its membership '
                'is reported descriptively.'),
        },
    }


def load_region(path):
    """Read and validate a region JSON file."""
    with open(path, 'r', encoding='utf-8') as handle:
        payload = json.load(handle)
    region = payload.get('region') or {}
    for key in ('min_radius_m', 'max_radius_m'):
        if key not in region:
            raise ValueError('region is missing {}'.format(key))
    for key in ('min_bearing_deg', 'max_bearing_deg'):
        if key not in region:
            raise ValueError('region is missing {}'.format(key))
    if not 0.15 <= float(region['min_radius_m']) < float(
            region['max_radius_m']) <= 0.65:
        raise ValueError('region radii are outside the supported range')
    if not -45.0 <= float(region['min_bearing_deg']) < float(
            region['max_bearing_deg']) <= 45.0:
        raise ValueError('region bearings are outside the supported range')
    if not payload.get('n_pairs') or not payload.get('seed'):
        raise ValueError('region needs n_pairs and seed')
    if payload.get('fixtures') != [
            fixture_id(index) for index in range(int(payload['n_pairs']))]:
        raise ValueError('region fixtures do not match n_pairs')
    return payload


def write_json(path, payload):
    """Write a JSON file, creating its directory."""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.',
                exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2)
        handle.write('\n')


def write_plan(path, region):
    """Write the seeded pair/method plan with the random fixtures."""
    from armpi_analysis import paired_trials

    pair_ids = ['p{:03d}'.format(index + 1)
                for index in range(int(region['n_pairs']))]
    plan = paired_trials.build_plan(
        pair_ids, fixtures=list(region['fixtures']),
        seed=int(region['seed']))
    write_json(path, plan)
    return plan


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--n-pairs', type=int, default=DEFAULT_N_PAIRS)
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED)
    parser.add_argument('--min-radius', type=float,
                        default=DEFAULT_MIN_RADIUS_M)
    parser.add_argument('--max-radius', type=float,
                        default=DEFAULT_MAX_RADIUS_M)
    parser.add_argument('--min-bearing', type=float,
                        default=DEFAULT_MIN_BEARING_DEG)
    parser.add_argument('--max-bearing', type=float,
                        default=DEFAULT_MAX_BEARING_DEG)
    parser.add_argument(
        '--output',
        default='evidence/phase3/onsite_20261121_random/region.json')
    parser.add_argument(
        '--plan',
        default='evidence/phase3/onsite_20261121_random/'
                'paired_trials_plan.json')
    return parser


def main(argv=None):
    """Build and write the region and the plan."""
    args = build_parser().parse_args(argv)
    region = build_region(
        n_pairs=args.n_pairs, seed=args.seed,
        min_radius=args.min_radius, max_radius=args.max_radius,
        min_bearing=args.min_bearing, max_bearing=args.max_bearing)
    write_json(args.output, region)
    print('wrote region to {}'.format(args.output))
    plan = write_plan(args.plan, region)
    print('wrote {} pairs / {} entries to {}'.format(
        plan['n_pairs'], len(plan['entries']), args.plan))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
