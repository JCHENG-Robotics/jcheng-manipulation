"""Tests for the Phase 3 measurement schedules and analyzers."""

import json

import pytest

import measurement_protocol as protocol


def test_summarize_values_known_sample():
    """Mean, sample standard deviation and range are computed correctly."""
    summary = protocol.summarize_values([0.0, 1.0, 2.0])
    assert summary['n'] == 3
    assert summary['mean'] == pytest.approx(1.0)
    assert summary['std'] == pytest.approx(1.0)
    assert summary['range'] == pytest.approx(2.0)


def test_repeatability_recovers_synthetic_noise():
    """Radial RMS is close to sigma*sqrt(3) for three-axis Gaussian noise."""
    samples = protocol._simulate_repeatability(
        [0.2, 0.0, 0.1], 200, noise_mm=0.3, bias_mm=0.0, seed=3
    )
    report = protocol.analyze_repeatability(samples, nominal=[0.2, 0.0, 0.1])
    assert 0.35 < report['summary']['radial_rms_mm'] < 0.75
    assert report['summary']['n'] == 200


def test_backlash_recovers_known_offset():
    """A 2 mm forward/reverse offset appears as 2 mm of hysteresis."""
    forward = [{'position_m': [0.200, 0.0, 0.1]} for _ in range(10)]
    reverse = [{'position_m': [0.198, 0.0, 0.1]} for _ in range(10)]
    report = protocol.analyze_backlash(
        forward, reverse, nominal=[0.2, 0.0, 0.1]
    )
    assert report['hysteresis']['magnitude_mm'] == pytest.approx(2.0)
    assert report['pass_2mm']


def test_visual_displacement_recovers_known_bias():
    """A 2 mm x bias appears in the per-axis and RMSE metrics."""
    records = [
        {
            'nominal_displacement_m': [0.01, 0.0, 0.0],
            'measured_displacement_m': [0.012, 0.0, 0.0],
        }
    ]
    report = protocol.analyze_visual_displacement(records)
    assert report['axes_mm']['x']['mean'] == pytest.approx(2.0)
    assert report['axes_mm']['y']['mean'] == pytest.approx(0.0)
    assert report['rmse_mm'] == pytest.approx(2.0)
    assert report['max_error_mm'] == pytest.approx(2.0)


def test_gripper_envelope_finds_single_axis_limit():
    """The largest passing offset on one axis sets the envelope."""
    records = []
    for axis in range(3):
        for offset in (-8.0, -6.0, 0.0, 6.0, 8.0):
            vector = [0.0, 0.0, 0.0]
            vector[axis] = offset / 1000.0
            records.append(
                {
                    'offset_m': vector,
                    'trials': 10,
                    'successes': 10 if abs(offset) <= 6.0 else 0,
                }
            )
    report = protocol.analyze_gripper_envelope(records)
    assert report['envelope_mm'] == {
        'x': pytest.approx(6.0),
        'y': pytest.approx(6.0),
        'z': pytest.approx(6.0),
    }


def test_dry_run_writes_a_30_trial_schedule(tmp_path):
    """The default dry-run writes a schedule and does not import ROS."""
    output = tmp_path / 'repeatability.json'
    status = protocol.main(
        ['repeatability', '--json-out', str(output)]
    )
    assert status == 0
    report = json.loads(output.read_text())
    assert report['dry_run']
    assert report['no_hardware']
    assert len(report['schedule']['entries']) == 30
    assert report['analysis'] is None


def test_invalid_target_joint_is_rejected(tmp_path):
    """A target outside the provisional limits fails before any schedule."""
    output = tmp_path / 'invalid.json'
    status = protocol.main(
        [
            'repeatability',
            '--target-joints',
            '0.0',
            '0.5',
            '1.0',
            '0.0',
            '0.0',
            '--json-out',
            str(output),
        ]
    )
    assert status == 2
    assert not output.exists()


@pytest.mark.parametrize(
    'protocol_name',
    ('repeatability', 'backlash', 'visual_displacement', 'gripper_envelope'),
)
def test_simulate_modes_analyze(protocol_name, tmp_path):
    """Every protocol's mock simulation produces an analysis block."""
    output = tmp_path / (protocol_name + '.json')
    status = protocol.main(
        [protocol_name, '--mode', 'simulate', '--json-out', str(output)]
    )
    assert status == 0
    report = json.loads(output.read_text())
    assert report['analysis'] is not None
    assert report['mode'] == 'simulate'


def test_analyze_reads_saved_samples(tmp_path):
    """A saved sample file can be analyzed without ROS."""
    measurement = {
        'nominal': [0.2, 0.0, 0.1],
        'samples': [
            {'position_m': [0.200, 0.000, 0.100]},
            {'position_m': [0.201, 0.000, 0.100]},
        ],
    }
    source = tmp_path / 'measurement.json'
    source.write_text(json.dumps(measurement))
    output = tmp_path / 'analysis.json'
    status = protocol.main(
        [
            'repeatability',
            '--mode',
            'analyze',
            '--input',
            str(source),
            '--json-out',
            str(output),
        ]
    )
    assert status == 0
    report = json.loads(output.read_text())
    assert report['analysis']['summary']['n'] == 2
    assert not report['dry_run']
