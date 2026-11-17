"""PEP 257 docstring check for the package."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Fail on missing or malformed docstrings."""
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found docstring errors'
