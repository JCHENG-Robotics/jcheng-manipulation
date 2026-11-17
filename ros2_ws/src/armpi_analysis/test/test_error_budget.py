"""Tests for the error-budget aggregation."""

import json
import math
import os

from armpi_analysis.error_budget import (
    aggregate,
    demo_budget,
    load_budget,
    render_markdown,
    render_svg,
    template_budget,
    validate_budget,
)
import pytest

BASELINE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config',
    'error_budget_baseline.json',
)


def _budget(sources, tolerance=None):
    return {
        'schema_version': 1,
        'name': 'test',
        'sources': sources,
        'tolerance': tolerance or {},
    }


def _source(**overrides):
    source = {
        'id': 's1',
        'category': 'calibration',
        'metric': 'translation_mm',
        'value': 1.0,
        'kind': 'bounded',
        'basis': 'max',
        'status': 'measured',
        'distribution': None,
    }
    source.update(overrides)
    return source


def test_measured_source_needs_value():
    """A measured source with no value is rejected."""
    with pytest.raises(ValueError):
        validate_budget(_budget([_source(value=None)]))


def test_unmeasured_source_rejects_value():
    """An unmeasured source cannot carry a value."""
    with pytest.raises(ValueError):
        validate_budget(
            _budget([_source(status='unmeasured', value=1.0)])
        )


def test_systematic_source_rejects_distribution():
    """A systematic source cannot be sampled."""
    with pytest.raises(ValueError):
        validate_budget(
            _budget([_source(kind='systematic', distribution='normal')])
        )


def test_normal_source_needs_sigma_or_basis():
    """A normal draw needs sigma or a std/ci95 basis."""
    with pytest.raises(ValueError):
        validate_budget(
            _budget(
                [_source(distribution='normal', basis='max', sigma=None)]
            )
        )


def test_duplicate_source_id_rejected():
    """Source identifiers are unique."""
    with pytest.raises(ValueError):
        validate_budget(_budget([_source(), _source()]))


def test_aggregate_rss_and_worst_case():
    """The demo budget has a known RSS and linear worst case."""
    report = aggregate(demo_budget(), samples=2000, seed=3)
    translation = report['metrics']['translation_mm']
    rotation = report['metrics']['rotation_deg']
    assert translation['rss'] == pytest.approx(
        math.sqrt(2.0 ** 2 + 1.0 ** 2 + 0.5 ** 2 + 0.3 ** 2)
    )
    assert translation['worst_case'] == pytest.approx(3.8)
    assert rotation['rss'] == pytest.approx(
        math.sqrt(0.4 ** 2 + 0.2 ** 2)
    )
    assert report['complete'] is True
    assert translation['pass_rss'] is True
    assert translation['pass_worst_case'] is True
    assert translation['dominant'] == 'calibration_translation'


def test_demo_interval_is_ordered():
    """The Monte Carlo interval brackets its own mean."""
    report = aggregate(demo_budget(), samples=5000, seed=11)
    interval = report['metrics']['translation_mm']['ci95']
    assert interval['low'] <= interval['mean'] <= interval['high']
    assert interval['high'] - interval['low'] > 0.0


def test_partial_baseline_lists_missing_sources():
    """The checked-in baseline is explicitly partial."""
    budget = load_budget(BASELINE)
    report = aggregate(budget, samples=1000)
    translation = report['metrics']['translation_mm']
    assert report['status'] == 'partial'
    assert report['complete'] is False
    assert translation['rss'] == pytest.approx(
        math.sqrt(3.29 ** 2 + 1.69 ** 2)
    )
    assert translation['worst_case'] == pytest.approx(4.98)
    assert translation['pass_rss'] is True
    assert translation['pass_worst_case'] is True
    assert 'arm_repeatability' in translation['missing_required']
    assert translation['ci95'] is None


def test_tolerance_failure_is_reported():
    """A source above tolerance fails the RSS gate."""
    budget = _budget(
        [_source(value=6.0)], tolerance={'translation_mm': 5.0}
    )
    report = aggregate(budget)
    assert report['metrics']['translation_mm']['pass_rss'] is False
    assert report['metrics']['translation_mm']['margin'] == pytest.approx(-1.0)


def test_template_validates():
    """The emitted template is a valid budget."""
    validated = validate_budget(template_budget())
    assert validated['name'] == 'replace_me'
    assert validated['sources'][0]['status'] == 'measured'


def test_markdown_marks_partial_budget():
    """The Markdown report warns when required sources are unmeasured."""
    report = aggregate(load_budget(BASELINE), samples=500)
    text = render_markdown(report)
    assert 'Partial budget' in text
    assert 'hand_eye_translation' in text
    assert 'Missing required sources' in text


def test_svg_contains_contributions_and_missing_sources():
    """The SVG chart names sources and marks missing required terms."""
    report = aggregate(load_budget(BASELINE), samples=500)
    svg = render_svg(report)
    assert svg.startswith('<svg')
    assert svg.rstrip().endswith('</svg>')
    assert 'hand_eye_translation' in svg
    assert 'missing required' in svg
    assert 'fiducial_grasp_partial_20261117' in svg


def test_report_is_json_serializable():
    """The aggregate report can be written as JSON."""
    report = aggregate(demo_budget(), samples=500)
    text = json.dumps(report)
    assert 'synthetic_error_budget_demo' in text
