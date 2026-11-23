"""Unit tests for the pure-Python safety policy."""

import math
from pathlib import Path

from armpi_safety.policy import (
    ARM_JOINTS,
    SafetyConfig,
    SafetyMonitor,
    load_config,
)
import pytest


def make_config(**overrides):
    """Return a deterministic test configuration."""
    raw = {
        'joint_limits': {
            joint: {
                'lower': -2.09,
                'upper': 2.09 if joint != 'joint3' else 0.0,
                'max_velocity': 1.2,
                'max_acceleration': 0.5,
            }
            for joint in ARM_JOINTS
        },
        'state': {'timeout_s': 0.5},
        'start_state': {'max_distance_rad': 0.08},
        'trajectory': {
            'max_duration_s': 30.0,
            'max_points': 100,
            'position_tolerance_rad': 1e-6,
            'velocity_tolerance_rad_s': 1e-6,
            'acceleration_tolerance_rad_s2': 1e-6,
        },
        'workspace': {
            'frame': 'base_link',
            'enabled': True,
            'required': True,
            'lower': [-0.05, -0.30, 0.00],
            'upper': [0.45, 0.30, 0.55],
        },
    }
    raw.update(overrides)
    return SafetyConfig.from_dict(raw)


def test_moveit_profile_is_the_single_rate_authority():
    raw = {
        'profiles': {
            'moveit': {
                'max_velocity_rad_s': 1.2,
                'max_acceleration_rad_s2': 0.5,
            }
        },
        'joint_limits': {
            joint: {
                'lower': -2.09,
                'upper': 0.0 if joint == 'joint3' else 2.09,
                'max_velocity': 1.3,
                'max_acceleration': 0.5,
            }
            for joint in ARM_JOINTS
        },
    }
    with pytest.raises(ValueError):
        SafetyConfig.from_dict(raw)


def test_shipped_config_matches_the_moveit_profile():
    path = Path(__file__).resolve().parents[1] / 'config/safety_limits.yaml'
    config = load_config(path)
    for joint in ARM_JOINTS:
        limit = config.joint_limits[joint]
        assert limit.max_velocity == pytest.approx(1.2)
        assert limit.max_acceleration == pytest.approx(0.5)


def trajectory(positions=None, times=None, velocities=None,
               accelerations=None):
    """Build a policy-schema trajectory from joint columns."""
    start = [0.0, 0.5, -1.0, -0.5, 0.0]
    if positions is None:
        positions = [start] * 5
    if times is None:
        times = [0.5 * index for index in range(len(positions))]
    points = []
    for index, point in enumerate(positions):
        points.append(
            {
                'positions': [float(value) for value in point],
                'velocities': (
                    [float(value) for value in velocities[index]]
                    if velocities else []
                ),
                'accelerations': (
                    [float(value) for value in accelerations[index]]
                    if accelerations else []
                ),
                'time_from_start_s': float(times[index]),
            }
        )
    return {'joint_names': list(ARM_JOINTS), 'points': points}


def monitor_with_state(config=None, positions=None, received_at=0.0):
    """Create a monitor whose latest state matches the trajectory start."""
    monitor = SafetyMonitor(
        config or make_config(), clock=lambda: received_at
    )
    monitor.update_joint_state(
        ARM_JOINTS,
        positions or [0.0, 0.5, -1.0, -0.5, 0.0],
        received_at=received_at,
    )
    return monitor


def fk_in_workspace(positions):
    """Fake FK inside the configured workspace."""
    del positions
    return (0.20, 0.00, 0.10)


def test_valid_trajectory_is_approved():
    """A slow, in-limit, in-workspace trajectory passes."""
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(), now=0.1, tcp_fk=fk_in_workspace
    )
    assert report['approved']
    assert report['code'] == 'OK'
    assert report['workspace_checked']
    assert report['max_velocity_rad_s'] <= 1.2
    assert report['max_acceleration_rad_s2'] <= 0.5


def test_stale_state_is_a_stop_condition():
    """A stale joint state vetoes a trajectory and raises the stop flag."""
    monitor = monitor_with_state(received_at=0.0)
    report = monitor.validate_trajectory(
        trajectory(), now=1.0, tcp_fk=fk_in_workspace
    )
    assert not report['approved']
    assert report['code'] == 'STATE_INVALID'
    assert monitor.status(now=1.0)['stop']


def test_joint_position_limit_is_rejected():
    """A waypoint outside the provisional joint range is rejected."""
    positions = [[0.0, 0.5, -1.0, -0.5, 0.0]] * 4 + [
        [2.5, 0.5, -1.0, -0.5, 0.0]
    ]
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(positions=positions), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'JOINT_LIMIT'
    assert any('joint1' in message for message in report['messages'])


def test_derived_velocity_limit_is_rejected():
    """A fast implicit move is caught with finite differences."""
    positions = [
        [1.0, 0.5, -1.0, -0.5, 0.0],
        [1.8, 0.5, -1.0, -0.5, 0.0],
    ]
    times = [0.0, 0.1]
    monitor = monitor_with_state(
        positions=[1.0, 0.5, -1.0, -0.5, 0.0]
    )
    report = monitor.validate_trajectory(
        trajectory(positions=positions, times=times), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'VELOCITY_LIMIT'


def test_explicit_velocity_limit_is_rejected():
    """Explicit velocities are checked even when positions move slowly."""
    velocities = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(velocities=velocities), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'VELOCITY_LIMIT'


def test_explicit_acceleration_limit_is_rejected():
    """Explicit trajectory accelerations are checked."""
    accelerations = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [3.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(accelerations=accelerations), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'ACCELERATION_LIMIT'


def test_start_state_mismatch_is_rejected():
    """The first point must match the latest measured state."""
    positions = [[0.5, 0.5, -1.0, -0.5, 0.0]] * 3
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(positions=positions), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'START_STATE'


def test_workspace_violation_is_rejected():
    """A tool point outside the workspace box is rejected."""
    def outside(positions):
        del positions
        return (0.50, 0.00, 0.10)

    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(), now=0.1, tcp_fk=outside
    )
    assert not report['approved']
    assert report['code'] == 'WORKSPACE'


def test_missing_workspace_fk_fails_closed():
    """Workspace checking is required, so no FK means no approval."""
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(trajectory(), now=0.1)
    assert not report['approved']
    assert report['code'] == 'WORKSPACE'


def test_missing_arm_joint_is_rejected():
    """Every arm joint must be present in the trajectory."""
    data = trajectory()
    data['joint_names'] = list(ARM_JOINTS[:-1])
    data['points'] = [
        {
            'positions': point['positions'][:-1],
            'velocities': [],
            'accelerations': [],
            'time_from_start_s': point['time_from_start_s'],
        }
        for point in data['points']
    ]
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        data, now=0.1, tcp_fk=fk_in_workspace
    )
    assert not report['approved']
    assert report['code'] == 'TRAJECTORY_INVALID'
    assert any('joint5' in message for message in report['messages'])


def test_non_monotonic_timestamps_are_rejected():
    """Trajectory time must strictly increase."""
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(times=[0.0, 0.5, 0.4, 1.0, 1.5]), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'TIMESTAMP'


def test_duration_limit_is_rejected():
    """A trajectory longer than the configured limit is rejected."""
    config = make_config(
        trajectory={
            'max_duration_s': 1.0,
            'max_points': 100,
            'position_tolerance_rad': 1e-6,
            'velocity_tolerance_rad_s': 1e-6,
            'acceleration_tolerance_rad_s2': 1e-6,
        }
    )
    monitor = monitor_with_state(config=config)
    report = monitor.validate_trajectory(
        trajectory(times=[0.0, 0.5, 1.0, 1.5, 2.0]), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'DURATION'


def test_reset_stop_requires_a_valid_state():
    """A latched stop clears only after a fresh valid state arrives."""
    monitor = monitor_with_state(received_at=0.0)
    assert monitor.status(now=1.0)['stop']
    result = monitor.reset_stop(now=1.0)
    assert not result['ok']
    monitor.update_joint_state(
        ARM_JOINTS, [0.0, 0.5, -1.0, -0.5, 0.0], received_at=1.0
    )
    result = monitor.reset_stop(now=1.1)
    assert result['ok']
    assert not monitor.status(now=1.1)['stop']


def test_non_finite_positions_are_rejected():
    """A non-finite position cannot pass the limits check."""
    positions = [[0.0, 0.5, -1.0, -0.5, 0.0]] * 2
    positions[1] = [math.nan, 0.5, -1.0, -0.5, 0.0]
    monitor = monitor_with_state()
    report = monitor.validate_trajectory(
        trajectory(positions=positions), now=0.1,
        tcp_fk=fk_in_workspace,
    )
    assert not report['approved']
    assert report['code'] == 'JOINT_LIMIT'


def test_config_defaults_and_invalid_workspace():
    """Default limits fill missing joints and bad bounds are rejected."""
    config = SafetyConfig.from_dict({})
    assert set(config.joint_limits) == set(ARM_JOINTS)
    assert config.joint_limits['joint3'].upper == 0.0
    assert not config.table_plane_enabled
    with pytest.raises(ValueError):
        SafetyConfig.from_dict(
            {'workspace': {'lower': [1.0, 0.0, 0.0],
                           'upper': [0.0, 1.0, 1.0]}}
        )


def table_config(**overrides):
    """Return a config with the table check and a lowered z hard floor."""
    raw = {
        'workspace': {
            'frame': 'base_link',
            'enabled': True,
            'required': True,
            'lower': [-0.05, -0.30, -0.10],
            'upper': [0.45, 0.30, 0.55],
        },
        'table_plane': {
            'enabled': True,
            'frame': 'base_link',
            'normal': [0.0, 0.0, 1.0],
            'offset': 0.078,
            'clearance_m': 0.01,
        },
    }
    raw.update(overrides)
    return make_config(**raw)


def test_table_plane_accepts_a_table_grasp_height():
    """A TCP 28 mm above the table passes the clearance check."""
    def fk(positions):
        del positions
        return (0.20, 0.00, -0.05)

    monitor = monitor_with_state(config=table_config())
    report = monitor.validate_trajectory(trajectory(), now=0.1, tcp_fk=fk)
    assert report['approved']
    assert report['table_checked']
    assert report['table_plane']['clearance_m'] == pytest.approx(0.01)


def test_table_plane_rejects_a_point_inside_clearance():
    """A TCP 3 mm above the table is inside the 10 mm clearance."""
    def fk(positions):
        del positions
        return (0.20, 0.00, -0.075)

    monitor = monitor_with_state(config=table_config())
    report = monitor.validate_trajectory(trajectory(), now=0.1, tcp_fk=fk)
    assert not report['approved']
    assert report['code'] == 'TABLE_CLEARANCE'
    violation = next(
        entry for entry in report['violations']
        if entry['check'] == 'table'
    )
    assert violation['height_m'] == pytest.approx(0.003, abs=1e-9)


def test_table_plane_uses_the_tilted_plane():
    """The same z can pass on the low side and fail on the high side."""
    config = table_config(table_plane={
        'enabled': True,
        'frame': 'base_link',
        'normal': [0.0, -0.063, 0.998],
        'offset': 0.078,
        'clearance_m': 0.01,
    })

    def high(positions):
        del positions
        return (0.20, 0.20, -0.070)

    def low(positions):
        del positions
        return (0.20, -0.20, -0.070)

    monitor = monitor_with_state(config=config)
    high_report = monitor.validate_trajectory(
        trajectory(), now=0.1, tcp_fk=high
    )
    low_report = monitor.validate_trajectory(
        trajectory(), now=0.1, tcp_fk=low
    )
    assert high_report['code'] == 'TABLE_CLEARANCE'
    assert low_report['approved']


def test_table_plane_requires_an_upward_normal():
    """A downward normal would invert the clearance test."""
    with pytest.raises(ValueError):
        make_config(table_plane={'enabled': True, 'normal': [0.0, 0.0, -1.0]})


def test_table_plane_requires_a_matching_frame():
    """The policy has no transform, so the frames must agree."""
    with pytest.raises(ValueError):
        make_config(table_plane={'enabled': True, 'frame': 'world'})


def test_table_plane_missing_fk_fails_closed():
    """The table check cannot approve without FK even if the box is off."""
    config = table_config(workspace={
        'frame': 'base_link',
        'enabled': False,
        'required': False,
        'lower': [-0.05, -0.30, -0.10],
        'upper': [0.45, 0.30, 0.55],
    })
    monitor = monitor_with_state(config=config)
    report = monitor.validate_trajectory(trajectory(), now=0.1)
    assert not report['approved']
    assert report['code'] == 'TABLE_CLEARANCE'
