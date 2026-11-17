"""Integration test for the phase-4 benchmark collector."""

import json
import os
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
RUNNER = REPO / 'tools' / 'phase4' / 'run_benchmark.sh'


def test_runner_syntax():
    """The collector script parses as bash."""
    result = subprocess.run(
        ['bash', '-n', str(RUNNER)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_runner_laptop_only(tmp_path):
    """A laptop-only run writes a host report without the robot."""
    env = dict(os.environ)
    env['SKIP_ROBOT'] = '1'
    env['SKIP_LAPTOP_ROS'] = '1'
    result = subprocess.run(
        [str(RUNNER), str(tmp_path)],
        capture_output=True,
        text=True,
        env=env,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    output = tmp_path / 'laptop_host.json'
    assert output.exists()
    payload = json.loads(output.read_text(encoding='utf-8'))
    assert payload['mode'] == 'host'
    assert payload['cpu']['logical_cores'] >= 1
