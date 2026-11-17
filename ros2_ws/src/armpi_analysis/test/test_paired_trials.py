"""Tests for paired-trial planning, recording and analysis."""

import json

from armpi_analysis.paired_trials import (
    analyze,
    append_record,
    build_plan,
    load_records,
    main,
    render_markdown,
    validate_dataset,
    validate_record,
)
import pytest


def record(pair_id, method, trial_order, error, success=True,
           failure_code=None, fixture='fixture_001'):
    """Build one valid trial record for a test."""
    return {
        'pair_id': pair_id,
        'trial_order': trial_order,
        'method': method,
        'object_pose_fixture': fixture,
        'continuous_error_mm': error,
        'grasp_success': success,
        'failure_code': failure_code,
    }


def pair(pair_id, baseline_error, intervention_error, baseline_success=True,
         intervention_success=True, baseline_first=True):
    """Build one complete matched pair."""
    if baseline_first:
        baseline_order, intervention_order = 1, 2
    else:
        baseline_order, intervention_order = 2, 1
    return [
        record(
            pair_id,
            'baseline',
            baseline_order,
            baseline_error,
            success=baseline_success,
            failure_code=None if baseline_success else 'MISS',
        ),
        record(
            pair_id,
            'intervention',
            intervention_order,
            intervention_error,
            success=intervention_success,
            failure_code=None if intervention_success else 'SLIP',
        ),
    ]


def test_success_without_error_is_rejected():
    """A successful grasp needs a continuous error."""
    with pytest.raises(ValueError):
        validate_record(record('p1', 'baseline', 1, None, success=True))


def test_success_with_failure_code_is_rejected():
    """A successful grasp cannot carry a failure code."""
    with pytest.raises(ValueError):
        validate_record(
            record('p1', 'baseline', 1, 1.0, success=True,
                   failure_code='MISS')
        )


def test_failure_needs_code():
    """A failed grasp needs a failure code."""
    with pytest.raises(ValueError):
        validate_record(
            record('p1', 'baseline', 1, None, success=False,
                   failure_code=None)
        )


def test_failure_may_have_null_error():
    """A failed grasp may have no measured displacement."""
    validated = validate_record(
        record('p1', 'baseline', 1, None, success=False,
               failure_code='MISS')
    )
    assert validated['continuous_error_mm'] is None


def test_dataset_groups_complete_and_incomplete():
    """Complete and incomplete pairs are reported separately."""
    records = pair('p1', 3.0, 2.0) + pair('p2', 3.0, 2.0)
    records.append(record('p3', 'baseline', 1, 2.5))
    report = validate_dataset(records)
    assert report['n_invalid'] == 0
    assert report['complete_pairs'] == ['p1', 'p2']
    assert report['incomplete_pairs'][0]['pair_id'] == 'p3'


def test_duplicate_pair_method_is_invalid():
    """The same pair/method cannot appear twice."""
    records = pair('p1', 3.0, 2.0) + [record('p1', 'baseline', 3, 2.0)]
    report = validate_dataset(records)
    assert report['n_invalid'] == 1
    assert 'duplicate' in report['invalid'][0]['error']


def test_plan_is_reproducible():
    """The same seed reproduces the randomized plan."""
    first = build_plan(['p1', 'p2', 'p3'], seed=5)
    second = build_plan(['p1', 'p2', 'p3'], seed=5)
    first_entries = [
        (
            entry['sequence'],
            entry['pair_id'],
            entry['trial_order'],
            entry['method'],
        )
        for entry in first['entries']
    ]
    second_entries = [
        (
            entry['sequence'],
            entry['pair_id'],
            entry['trial_order'],
            entry['method'],
        )
        for entry in second['entries']
    ]
    assert first_entries == second_entries


def test_plan_has_both_methods_per_pair():
    """Every pair is scheduled with one baseline and one intervention."""
    plan = build_plan(['p1', 'p2', 'p3', 'p4'], seed=5)
    for pair_entry in plan['pairs']:
        assert set(pair_entry['order']) == {'baseline', 'intervention'}
    assert len(plan['entries']) == 8


def test_analyze_recovers_known_effect():
    """A 1 mm paired improvement is recovered with a strong sign test."""
    records = []
    for index in range(30):
        records.extend(
            pair(
                'p{:03d}'.format(index),
                3.0,
                2.0,
                baseline_first=index % 2 == 0,
            )
        )
    report = analyze(records, bootstrap_samples=1000)
    paired = report['paired']
    assert paired['difference_mm']['mean'] == pytest.approx(-1.0)
    assert paired['improvement_mm'] == pytest.approx(1.0)
    assert paired['sign_test']['wins'] == 30
    assert paired['sign_test']['losses'] == 0
    assert paired['sign_test']['p_value'] < 1e-6
    assert report['randomization']['baseline_first'] == 15


def test_analyze_reports_mcnemar():
    """Five baseline-only successes give McNemar p=0.0625."""
    records = []
    for index in range(10):
        records.extend(
            pair(
                'p{:03d}'.format(index),
                2.0,
                2.0,
                baseline_success=True,
                intervention_success=index >= 5,
            )
        )
    report = analyze(records, bootstrap_samples=500)
    assert report['success']['discordant']['baseline_only'] == 5
    assert report['success']['discordant']['intervention_only'] == 0
    assert report['success']['baseline_rate'] == pytest.approx(1.0)
    assert report['success']['intervention_rate'] == pytest.approx(0.5)
    assert report['success']['mcnemar_exact_p'] == pytest.approx(0.0625)


def test_analyze_marks_order_groups():
    """The order-effect section separates baseline-first pairs."""
    records = pair('p1', 3.0, 2.5, baseline_first=True)
    records += pair('p2', 3.0, 3.0, baseline_first=False)
    report = analyze(records, bootstrap_samples=500)
    assert report['randomization']['baseline_first'] == 1
    assert report['order_effect']['groups']['baseline_first']['n'] == 1
    assert report['order_effect']['groups']['intervention_first']['n'] == 1
    assert report['order_effect']['difference_mm'] == pytest.approx(0.5)


def test_analyze_requires_a_complete_pair():
    """Analysis fails when no pair has both methods."""
    with pytest.raises(ValueError):
        analyze([record('p1', 'baseline', 1, 2.0)])


def test_append_record_enforces_plan_and_uniqueness(tmp_path):
    """Recording checks the plan and refuses duplicates."""
    plan = build_plan(['p1'], seed=3)
    entry = plan['entries'][0]
    other = plan['entries'][1]
    path = tmp_path / 'trials.jsonl'
    append_record(
        str(path),
        record(
            entry['pair_id'],
            entry['method'],
            entry['trial_order'],
            2.0,
            fixture=entry['object_pose_fixture'],
        ),
        plan=plan,
    )
    with pytest.raises(ValueError):
        append_record(
            str(path),
            record(
                entry['pair_id'],
                entry['method'],
                entry['trial_order'],
                2.0,
                fixture=entry['object_pose_fixture'],
            ),
            plan=plan,
        )
    with pytest.raises(ValueError):
        append_record(
            str(path),
            record(
                other['pair_id'],
                other['method'],
                other['trial_order'],
                2.0,
                fixture=other['object_pose_fixture'],
            ),
            plan=build_plan(['other_pair'], seed=3),
        )


def test_jsonl_roundtrip(tmp_path):
    """JSONL records load back without losing fields."""
    path = tmp_path / 'trials.jsonl'
    records = pair('p1', 3.0, 2.0)
    for item in records:
        append_record(str(path), item)
    loaded = load_records(str(path))
    assert len(loaded) == 2
    assert loaded[0]['pair_id'] == 'p1'


def test_markdown_contains_sections():
    """The Markdown report includes the main analysis sections."""
    report = analyze(pair('p1', 3.0, 2.0), bootstrap_samples=500)
    text = render_markdown(report)
    assert '# Paired Trial Analysis' in text
    assert '## Paired continuous error' in text
    assert 'McNemar' in text
    assert '## Failure codes' in text


def test_simulate_writes_analysis(tmp_path):
    """The simulation exercises planning, recording and analysis."""
    output = tmp_path / 'simulation.json'
    jsonl = tmp_path / 'trials.jsonl'
    code = main(
        [
            'simulate',
            '--pairs',
            '20',
            '--seed',
            '7',
            '--json-out',
            str(output),
            '--jsonl-out',
            str(jsonl),
        ]
    )
    assert code == 0
    payload = json.loads(output.read_text(encoding='utf-8'))
    assert payload['analysis']['validation']['n_complete_pairs'] == 20
    assert len(load_records(str(jsonl))) == 40
