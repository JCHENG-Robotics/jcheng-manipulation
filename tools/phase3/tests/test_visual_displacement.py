"""Tests for the visual-displacement runner (no robot, no images)."""

import json
import math
import os
import sys

import numpy as np

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2')):
    if path not in sys.path:
        sys.path.insert(0, path)

import visual_displacement_run as vd  # noqa: E402


def test_schedule_has_an_origin_at_each_block_boundary():
    schedule = vd.build_schedule(10, 20, 2, 7)
    # 4 offsets per direction pair x2 axes = 8 offsets, plus 2 origins.
    assert len(schedule) == 20
    for block in (1, 2):
        block_entries = [entry for entry in schedule if entry[0] == block]
        assert block_entries[0] == (block, 0, 0)
        assert block_entries[-1] == (block, 0, 0)
        offsets = [(u, v) for _, u, v in block_entries[1:-1]]
        assert sorted(offsets) == sorted([
            (-20, 0), (20, 0), (-10, 0), (10, 0),
            (0, -20), (0, 20), (0, -10), (0, 10),
        ])


def test_schedule_is_seeded():
    assert (vd.build_schedule(10, 20, 2, 7)
            == vd.build_schedule(10, 20, 2, 7))
    assert (vd.build_schedule(10, 20, 2, 7)
            != vd.build_schedule(10, 20, 2, 8))


def test_fit_ruler_rotation_recovers_a_known_yaw():
    nominal = np.array([[30.0, 0.0], [0.0, 30.0], [-20.0, 10.0]])
    angle = math.radians(12.0)
    measured = vd.rotate_2d(nominal, angle)
    fitted = vd.fit_ruler_rotation(nominal, measured)
    assert abs(fitted - angle) < 1e-12


def test_rotate_2d_is_a_proper_rotation():
    rotated = vd.rotate_2d([[1.0, 0.0]], math.pi / 2.0)
    assert np.allclose(rotated, [[0.0, 1.0]])


def test_dry_run_recovers_the_synthetic_ruler_yaw(tmp_path):
    positions = tmp_path / 'positions.json'
    analysis_path = tmp_path / 'analysis.json'
    rc = vd.main([
        '--dry-run', '--repeats', '1', '--step-mm', '10', '--max-mm', '20',
        '--positions-out', str(positions),
        '--analysis-out', str(analysis_path),
    ])
    assert rc == 0
    data = json.loads(analysis_path.read_text())
    # The dry positions apply a -12 deg ruler yaw; the fit removes it.
    assert abs(data['ruler_yaw_deg'] + 12.0) < 1e-9
    assert data['rotation_fit']['residual_max_mm'] < 1e-9
    assert data['records']
