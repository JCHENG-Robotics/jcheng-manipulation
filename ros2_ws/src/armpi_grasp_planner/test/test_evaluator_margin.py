"""Tests for the evaluator's joint-limit margin wiring (no MoveIt needed)."""

from types import SimpleNamespace

from armpi_grasp_planner.candidates import generate_candidates
from armpi_grasp_planner.evaluator import MoveItGraspEvaluator
import pytest

ARM_LIMITS = {
    'joint1': (-1.8012, 1.8012),
    'joint2': (-0.6618, 1.1519),
    'joint3': (-1.8012, 0.0),
    'joint4': (-1.5917, 1.5917),
    'joint5': (-1.5917, 1.5499),
}


def _candidate():
    return generate_candidates(
        (0.1, 0.0, 0.1),
        pitches_deg=[30.0],
        approach_yaws_deg=[0.0],
        rolls_deg=[0.0],
        standoffs_m=[0.0],
    )[0]


def _fake_evaluator(compute_ik, **overrides):
    attributes = {
        'compute_ik': compute_ik,
        'check_fk': lambda *args: pytest.fail('FK must not run'),
        'check_collision': lambda *args: pytest.fail(
            'collision must not run'
        ),
        'plan': lambda *args, **kwargs: pytest.fail('plan must not run'),
        'safety_service': '',
        'last_planned_trajectory': None,
    }
    attributes.update(overrides)
    return SimpleNamespace(**attributes)


def test_evaluate_all_rejects_ik_solution_on_a_limit():
    """An IK solution exactly on joint2's bound is rejected, not planned."""
    def compute_ik(pose, seed, timeout):
        return {
            'ok': True,
            'code': 1,
            'name': 'SUCCESS',
            'joints': [0.0, -0.6618, 0.0, 0.0, 0.0],
            'elapsed_s': 0.0,
        }

    results, counts = MoveItGraspEvaluator.evaluate_all(
        _fake_evaluator(compute_ik),
        [_candidate()],
        [(0.0, 0.0, 0.0, 0.0, 0.0)],
        max_plan=1,
        joint_limits=ARM_LIMITS,
        joint_limit_margin_rad=0.03,
    )
    assert results[0]['status'] == 'JOINT_LIMIT_MARGIN'
    assert results[0]['joint_margin']['min_margin_rad'] == pytest.approx(
        0.0, abs=1e-9
    )
    assert counts['margin_rejected'] == 1
    assert counts['plan_attempted'] == 0


def test_evaluate_all_clamps_seed_before_ik():
    """A seed outside the margin reaches IK clamped into the shrunk range."""
    seen = {}

    def compute_ik(pose, seed, timeout):
        seen['seed'] = seed
        return {
            'ok': True,
            'code': 1,
            'name': 'SUCCESS',
            'joints': [0.0, 0.0, -0.5, 0.0, 0.0],
            'elapsed_s': 0.0,
        }

    results, counts = MoveItGraspEvaluator.evaluate_all(
        _fake_evaluator(
            compute_ik,
            check_fk=lambda *args: {
                'ok': True,
                'position_error_m': 0.0,
                'orientation_error_deg': 0.0,
            },
            check_collision=lambda *args: {
                'ok': True,
                'contact_count': 0,
                'contacts': [],
            },
        ),
        [_candidate()],
        [(0.0, 0.7309, -1.6127, -1.6591, 0.0)],
        max_plan=0,
        joint_limits=ARM_LIMITS,
        joint_limit_margin_rad=0.03,
    )
    assert counts['seeds_clamped'] == 1
    assert seen['seed'][3] == pytest.approx(-1.5617)
    assert results[0]['status'] == 'COLLISION_FREE'
