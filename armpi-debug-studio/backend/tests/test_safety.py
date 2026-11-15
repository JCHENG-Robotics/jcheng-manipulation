from __future__ import annotations

import math

import pytest

from app.domain.enums import ErrorCode
from app.domain.mapping import MappingConfig
from app.domain.models import MotionTarget
from app.safety.validation import validate_targets


def _codes(outcome) -> list[str]:
    return [issue.code for issue in outcome.errors]


def test_valid_single_joint(mapping: MappingConfig) -> None:
    outcome = validate_targets(
        mapping,
        [MotionTarget("joint1", 0.1, 1.0)],
        current_targets={"joint1": 0.0},
    )
    assert outcome.valid
    assert outcome.normalized[0]["target_raw"] == 524
    assert any(issue.code == "MAPPING_PROVISIONAL" for issue in outcome.warnings)


def test_unknown_joint_rejected(mapping: MappingConfig) -> None:
    outcome = validate_targets(mapping, [MotionTarget("joint9", 0.1, 1.0)])
    assert not outcome.valid
    assert ErrorCode.UNKNOWN_JOINT.value in _codes(outcome)


def test_gripper_excluded(mapping: MappingConfig) -> None:
    outcome = validate_targets(mapping, [MotionTarget("r_joint", 0.1, 1.0)])
    assert not outcome.valid
    assert ErrorCode.GRIPPER_NOT_IN_G1.value in _codes(outcome)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_rejected(mapping: MappingConfig, bad: float) -> None:
    outcome = validate_targets(mapping, [MotionTarget("joint1", bad, 1.0)])
    assert not outcome.valid
    assert ErrorCode.NON_FINITE_TARGET.value in _codes(outcome)


def test_joint3_positive_rejected(mapping: MappingConfig) -> None:
    outcome = validate_targets(mapping, [MotionTarget("joint3", 0.1, 1.0)])
    assert not outcome.valid
    assert ErrorCode.OUT_OF_LIMIT.value in _codes(outcome)


def test_step_limit_rejected(mapping: MappingConfig) -> None:
    outcome = validate_targets(
        mapping,
        [MotionTarget("joint1", 0.5, 1.0)],
        current_targets={"joint1": 0.0},
    )
    assert not outcome.valid
    assert ErrorCode.STEP_EXCEEDED.value in _codes(outcome)


@pytest.mark.parametrize("bad_duration", [0.1, 11.0, math.nan])
def test_duration_range_rejected(mapping: MappingConfig, bad_duration: float) -> None:
    outcome = validate_targets(mapping, [MotionTarget("joint1", 0.1, bad_duration)])
    assert not outcome.valid
    assert ErrorCode.DURATION_RANGE.value in _codes(outcome)


def test_duplicate_joint_rejected(mapping: MappingConfig) -> None:
    outcome = validate_targets(
        mapping,
        [MotionTarget("joint1", 0.1, 1.0), MotionTarget("joint1", 0.2, 1.0)],
    )
    assert not outcome.valid
    assert ErrorCode.DUPLICATE_JOINT.value in _codes(outcome)


def test_empty_targets_rejected(mapping: MappingConfig) -> None:
    outcome = validate_targets(mapping, [])
    assert not outcome.valid
    assert ErrorCode.NO_TARGETS.value in _codes(outcome)


def test_joint3_negative_allowed(mapping: MappingConfig) -> None:
    outcome = validate_targets(
        mapping,
        [MotionTarget("joint3", -0.1, 1.0)],
        current_targets={"joint3": 0.0},
    )
    assert outcome.valid
    assert outcome.normalized[0]["target_raw"] == 524
