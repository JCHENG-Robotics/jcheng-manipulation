"""Flake8 lint check for the package."""

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Fail on flake8 errors."""
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, 'Found {} code style errors / warnings:\n{}'.format(
        len(errors), '\n'.join(errors)
    )
