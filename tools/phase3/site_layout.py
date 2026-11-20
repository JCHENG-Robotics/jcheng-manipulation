#!/usr/bin/env python3
"""Define the 24 floor sites of the site-diversity paired trial.

The 2026-11-19 paired trial placed the cube on one fixture. The follow-up
experiment asks whether the same method comparison survives when every pair
runs at its **own** floor site, so the robot has to locate the object, drive
the base to it and only then grasp.

The layout is a fan read from the validated ``view`` pose. Four radii and six
bearings give 24 distinct sites:

    radii    0.30, 0.35, 0.40, 0.45 m
    bearings -10, -4, +2, +8, +14, +20 deg

The bearings are deliberately asymmetric about the robot's forward axis: the
RealSense front camera is mounted at ``base_link`` y = +0.027 m with a -1.1
deg yaw, so a fan centred on the camera's optical axis spends more of the
camera's 35 deg half-field on the +y side. All 24 nominal top-face squares
project inside both camera images from the home/view pose (the tightest wrist
corner margin is about 20 px at 0.45 m; the front camera keeps more than
30 px). The robot therefore does not need a yaw search to see the nominal
sites; the yaw sweep remains a fallback for a coarse home return.

Sites are labelled radius-major: ``s01`` = (0.30 m, -10 deg), ``s06`` =
(0.30 m, +20 deg), ``s07`` = (0.35 m, -10 deg), ..., ``s24`` =
(0.45 m, +20 deg).

Run on the laptop (no ROS, no robot)::

    python3 tools/phase3/site_layout.py \
        --output evidence/phase3/onsite_20261120_sites/site_layout.json \
        --markdown evidence/phase3/onsite_20261120_sites/site_layout.md

The JSON is the pre-registered layout; the markdown table is the operator's
floor-taping sheet. Re-running the tool reproduces the same sites.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os

SCHEMA_VERSION = 1
DEFAULT_RADII_M = (0.30, 0.35, 0.40, 0.45)
DEFAULT_BEARINGS_DEG = (-10.0, -4.0, 2.0, 8.0, 14.0, 20.0)
#: The validated view-pose wrist image stops seeing a cube top near 0.46 m;
#: 0.47 m leaves a margin for the arm's measured sag.
MAX_RADIUS_M = 0.47
MIN_RADIUS_M = 0.25
#: The front camera's useful fan; beyond this the wrist needs a yaw search.
MIN_BEARING_DEG = -15.0
MAX_BEARING_DEG = 25.0
#: At 0.30 m, adjacent 6 deg bearings are 31 mm apart (the cube is 39.4 mm).
MIN_SITE_SEPARATION_M = 0.025
FIXTURE_PREFIX = 's'


def site_id(index):
    """Return the stable site id for a zero-based layout index."""
    return '{}{:02d}'.format(FIXTURE_PREFIX, index + 1)


def build_sites(radii=DEFAULT_RADII_M, bearings=DEFAULT_BEARINGS_DEG):
    """Return the site list, radius-major, with base-frame coordinates."""
    radii = [float(value) for value in radii]
    bearings = [float(value) for value in bearings]
    if not radii or not bearings:
        raise ValueError('radii and bearings must be non-empty')
    sites = []
    for radius in radii:
        for bearing in bearings:
            angle = math.radians(bearing)
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            index = len(sites)
            sites.append({
                'site_id': site_id(index),
                'radius_m': round(radius, 6),
                'bearing_deg': round(bearing, 6),
                'x_m': round(x, 6),
                'y_m': round(y, 6),
            })
    validate_sites(sites)
    return sites


def validate_sites(sites):
    """Refuse a layout that leaves the pre-registered envelope."""
    if not sites:
        raise ValueError('a layout needs at least one site')
    ids = [site['site_id'] for site in sites]
    if len(set(ids)) != len(ids):
        raise ValueError('site ids are not unique')
    for site in sites:
        radius = float(site['radius_m'])
        bearing = float(site['bearing_deg'])
        if not MIN_RADIUS_M <= radius <= MAX_RADIUS_M:
            raise ValueError(
                'site {} radius {:.3f} m is outside {}-{} m'.format(
                    site['site_id'], radius, MIN_RADIUS_M, MAX_RADIUS_M))
        if not MIN_BEARING_DEG <= bearing <= MAX_BEARING_DEG:
            raise ValueError(
                'site {} bearing {:.1f} deg is outside {}-{} deg'.format(
                    site['site_id'], bearing, MIN_BEARING_DEG,
                    MAX_BEARING_DEG))
        x, y = float(site['x_m']), float(site['y_m'])
        if not math.isclose(math.hypot(x, y), radius, abs_tol=1e-5):
            raise ValueError('site {} radius does not match x/y'.format(
                site['site_id']))
        if not math.isclose(
                math.degrees(math.atan2(y, x)), bearing, abs_tol=1e-4):
            raise ValueError('site {} bearing does not match x/y'.format(
                site['site_id']))
    for first in range(len(sites)):
        for second in range(first + 1, len(sites)):
            a = sites[first]
            b = sites[second]
            distance = math.hypot(
                float(a['x_m']) - float(b['x_m']),
                float(a['y_m']) - float(b['y_m']))
            if distance < MIN_SITE_SEPARATION_M:
                raise ValueError(
                    'sites {} and {} are {:.1f} mm apart (< {:.0f} mm)'.format(
                        a['site_id'], b['site_id'], distance * 1000.0,
                        MIN_SITE_SEPARATION_M * 1000.0))


def layout_payload(sites):
    """Return the JSON object written for the pre-registered layout."""
    return {
        'schema_version': SCHEMA_VERSION,
        'created_at': datetime.now(timezone.utc).isoformat(),
        'fixture_prefix': FIXTURE_PREFIX,
        'n_sites': len(sites),
        'radii_m': sorted({site['radius_m'] for site in sites}),
        'bearings_deg': sorted({site['bearing_deg'] for site in sites}),
        'envelope': {
            'min_radius_m': MIN_RADIUS_M,
            'max_radius_m': MAX_RADIUS_M,
            'min_bearing_deg': MIN_BEARING_DEG,
            'max_bearing_deg': MAX_BEARING_DEG,
            'min_site_separation_m': MIN_SITE_SEPARATION_M,
            'note': (
                'The nominally placed top-face squares project inside the '
                'view-pose wrist image (640x400) and the front D435i colour '
                'image (1280x720); the layout is centred on the front '
                'camera, whose optical axis is at base y = +0.027 m and '
                'yaw -1.1 deg.'),
        },
        'sites': sites,
    }


def write_json(path, payload):
    """Write the layout JSON, creating its directory."""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.',
                exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2)
        handle.write('\n')


def markdown_table(sites):
    """Return the floor-taping sheet for the operator."""
    lines = [
        '| Site | Radius (m) | Bearing (deg) | Forward x (m) | '
        'Left y (m) |',
        '|---|---:|---:|---:|---:|',
    ]
    for site in sites:
        lines.append('| {site_id} | {radius_m:.2f} | {bearing_deg:+.0f} | '
                     '{x_m:.3f} | {y_m:+.3f} |'.format(**site))
    return '\n'.join(lines) + '\n'


def write_text(path, text):
    """Write a text artifact, creating its directory."""
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.',
                exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(text)


def load_layout(path):
    """Read and validate a layout JSON file."""
    with open(path, 'r', encoding='utf-8') as handle:
        payload = json.load(handle)
    sites = payload.get('sites')
    if not isinstance(sites, list):
        raise ValueError('layout has no sites list')
    validate_sites(sites)
    if payload.get('n_sites') not in (None, len(sites)):
        raise ValueError('layout n_sites does not match the site list')
    return payload


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--radii', default=','.join('{:.2f}'.format(value)
                                    for value in DEFAULT_RADII_M),
        help='comma-separated radii in metres (default 0.30..0.45)')
    parser.add_argument(
        '--bearings', default=','.join('{:+.0f}'.format(value)
                                       for value in DEFAULT_BEARINGS_DEG),
        help='comma-separated bearings in degrees (default -10..+20)')
    parser.add_argument(
        '--output',
        default='evidence/phase3/onsite_20261120_sites/site_layout.json',
        help='layout JSON to write')
    parser.add_argument(
        '--markdown', default=None,
        help='optional operator markdown table to write')
    return parser


def main(argv=None):
    """Build, validate and write the layout."""
    args = build_parser().parse_args(argv)
    radii = [float(value) for value in args.radii.split(',') if value]
    bearings = [float(value) for value in args.bearings.split(',') if value]
    sites = build_sites(radii, bearings)
    payload = layout_payload(sites)
    write_json(args.output, payload)
    print('wrote {} sites to {}'.format(len(sites), args.output))
    if args.markdown:
        write_text(args.markdown, markdown_table(sites))
        print('wrote the operator table to {}'.format(args.markdown))
    print(markdown_table(sites))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
