"""Tests for the 24-site layout (no robot, no ROS)."""

import json
import math
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2'),
             os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis')):
    if path not in sys.path:
        sys.path.insert(0, path)

import site_layout as layout  # noqa: E402


def test_default_layout_has_24_unique_sites():
    sites = layout.build_sites()
    assert len(sites) == 24
    assert len({site['site_id'] for site in sites}) == 24
    assert {site['radius_m'] for site in sites} == {0.30, 0.35, 0.40, 0.45}
    assert {site['bearing_deg'] for site in sites} == {
        -10.0, -4.0, 2.0, 8.0, 14.0, 20.0}


def test_sites_are_radius_major_and_inside_the_envelope():
    sites = layout.build_sites()
    for site in sites:
        assert layout.MIN_RADIUS_M <= site['radius_m'] <= layout.MAX_RADIUS_M
        assert (layout.MIN_BEARING_DEG <= site['bearing_deg']
                <= layout.MAX_BEARING_DEG)
        radius = math.hypot(site['x_m'], site['y_m'])
        assert radius == pytest.approx(site['radius_m'], abs=1e-5)
        bearing = math.degrees(math.atan2(site['y_m'], site['x_m']))
        assert bearing == pytest.approx(site['bearing_deg'], abs=1e-4)
    assert sites[0]['site_id'] == 's01'
    assert sites[0]['radius_m'] == 0.30
    assert sites[0]['bearing_deg'] == -10.0
    assert sites[-1]['site_id'] == 's24'
    assert sites[-1]['radius_m'] == 0.45
    assert sites[-1]['bearing_deg'] == 20.0
    assert sites[-1]['x_m'] == pytest.approx(0.42286, abs=1e-4)
    assert sites[-1]['y_m'] == pytest.approx(0.15391, abs=1e-4)


def test_adjacent_sites_do_not_overlap():
    sites = layout.build_sites()
    for first in range(len(sites)):
        for second in range(first + 1, len(sites)):
            a, b = sites[first], sites[second]
            distance = math.hypot(
                a['x_m'] - b['x_m'], a['y_m'] - b['y_m'])
            assert distance >= layout.MIN_SITE_SEPARATION_M


def test_a_custom_layout_keeps_the_envelope_checks():
    sites = layout.build_sites((0.30, 0.40), (-5.0, 5.0))
    assert len(sites) == 4
    assert [site['site_id'] for site in sites] == [
        's01', 's02', 's03', 's04']
    with pytest.raises(ValueError):
        layout.build_sites((0.60,), (0.0,))
    with pytest.raises(ValueError):
        layout.build_sites((0.30,), (40.0,))
    with pytest.raises(ValueError):
        layout.build_sites((0.30,), (-20.0,))


def test_duplicate_sites_are_refused():
    duplicate = [
        {'site_id': 's01', 'radius_m': 0.30, 'bearing_deg': 0.0,
         'x_m': 0.30, 'y_m': 0.0},
        {'site_id': 's02', 'radius_m': 0.30, 'bearing_deg': 0.0,
         'x_m': 0.30, 'y_m': 0.0},
    ]
    with pytest.raises(ValueError):
        layout.validate_sites(duplicate)


def test_json_round_trip_and_markdown_rows(tmp_path):
    sites = layout.build_sites()
    path = tmp_path / 'layout.json'
    layout.write_json(path, layout.layout_payload(sites))
    payload = layout.load_layout(str(path))
    assert payload['n_sites'] == 24
    assert payload['sites'] == sites
    table = layout.markdown_table(sites)
    rows = [line for line in table.splitlines() if line.startswith('| s')]
    assert len(rows) == 24
    assert '| s24 | 0.45 | +20 | 0.423 | +0.154 |' in rows


def test_a_layout_with_the_wrong_site_count_is_refused(tmp_path):
    sites = layout.build_sites()
    payload = layout.layout_payload(sites)
    payload['n_sites'] = 23
    path = tmp_path / 'layout.json'
    path.write_text(json.dumps(payload), encoding='utf-8')
    with pytest.raises(ValueError):
        layout.load_layout(str(path))
