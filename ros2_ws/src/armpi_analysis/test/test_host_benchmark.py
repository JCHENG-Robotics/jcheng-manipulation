"""Tests for the read-only host/ROS benchmark helpers."""

import json

from armpi_analysis.host_benchmark import (
    _summary_ns,
    collect_host,
    main,
)
import pytest


def test_summary_ns_known_values():
    """Interval statistics use milliseconds and sample deviation."""
    summary = _summary_ns([1_000_000, 2_000_000, 3_000_000])
    assert summary['n'] == 3
    assert summary['mean_ms'] == pytest.approx(2.0)
    assert summary['std_ms'] == pytest.approx(1.0)
    assert summary['min_ms'] == pytest.approx(1.0)
    assert summary['max_ms'] == pytest.approx(3.0)
    assert summary['p95_ms'] == pytest.approx(3.0)


def test_summary_ns_empty():
    """An empty sample has null statistics."""
    summary = _summary_ns([])
    assert summary['n'] == 0
    assert summary['mean_ms'] is None


def test_collect_host_shape():
    """Host collection returns the documented fields."""
    report = collect_host(sample_s=0.05)
    assert report['mode'] == 'host'
    assert report['architecture']
    assert report['python']
    assert report['cpu']['logical_cores'] >= 1
    assert report['memory']['total_mb'] > 0
    assert report['disk']['total_gb'] > 0
    assert 'distro' in report['ros']
    if report['cpu']['utilization_percent'] is not None:
        assert 0.0 <= report['cpu']['utilization_percent'] <= 100.0
    json.dumps(report)


def test_collect_host_rejects_long_sample():
    """The CPU sample window is bounded."""
    with pytest.raises(ValueError):
        collect_host(sample_s=60.0)


def test_main_host_writes_json(tmp_path):
    """The host CLI writes a JSON report."""
    output = tmp_path / 'host.json'
    code = main(
        ['--mode', 'host', '--sample-s', '0.05', '--json-out', str(output)]
    )
    assert code == 0
    payload = json.loads(output.read_text(encoding='utf-8'))
    assert payload['mode'] == 'host'
    assert payload['hostname']
