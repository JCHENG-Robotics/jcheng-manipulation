"""Offline tests for the G5 capture buffer and dataset assembly."""

from __future__ import annotations

import ast
import math
from pathlib import Path

import pytest

from armpi_debug_gateway.trajectory_capture import (
    ARM_JOINTS,
    CaptureBuffer,
    CaptureError,
    build_dataset,
    build_static_planned,
    dataset_from_capture,
    load_planned_trajectory,
    pulse_to_rad,
    rad_to_pulse,
    select_planned_joints,
    servo_states_to_positions,
)


class FakeServoState:
    def __init__(self, servo_id: int, position: list[int] | int) -> None:
        self.id = servo_id
        self.position = position


def test_servo_states_accept_scalar_fields_from_vendor_message() -> None:
    states = [
        FakeServoState(6, 501),
        FakeServoState(5, 598),
        FakeServoState(4, 800),
        FakeServoState(3, 328),
        FakeServoState(2, 498),
    ]
    positions = servo_states_to_positions(states)
    assert positions["joint1"] == pytest.approx(0.0041887902)
    assert positions["joint3"] == pytest.approx(-1.2566370614)


def test_pulse_conversion_matches_measured_joint_states() -> None:
    # Values captured from /controller_manager/servo_states and joint_states
    # on 2026-11-18.
    assert pulse_to_rad("joint1", 501) == pytest.approx(0.0041887902)
    assert pulse_to_rad("joint2", 598) == pytest.approx(0.4105014401)
    assert pulse_to_rad("joint3", 800) == pytest.approx(-1.2566370614)
    assert pulse_to_rad("joint4", 328) == pytest.approx(-0.7204719152)
    assert pulse_to_rad("joint5", 498) == pytest.approx(-0.0083775804)
    for joint, pulse in (("joint1", 501), ("joint3", 800)):
        assert rad_to_pulse(joint, pulse_to_rad(joint, pulse)) == pytest.approx(pulse)


def test_servo_states_convert_and_require_all_joints() -> None:
    states = [
        FakeServoState(6, [501]),
        FakeServoState(5, [598]),
        FakeServoState(4, [800]),
        FakeServoState(3, [328]),
        FakeServoState(2, [498]),
        FakeServoState(1, [509]),
    ]
    positions = servo_states_to_positions(states)
    assert list(positions) == list(ARM_JOINTS)
    assert positions["joint3"] == pytest.approx(-1.2566370614)

    with pytest.raises(CaptureError) as missing:
        servo_states_to_positions(states[:4])
    assert "servo 2 missing" in str(missing.value.issues)


def test_out_of_range_pulse_is_rejected_not_converted() -> None:
    states = [
        FakeServoState(6, [1164]),
        FakeServoState(5, [598]),
        FakeServoState(4, [800]),
        FakeServoState(3, [328]),
        FakeServoState(2, [498]),
    ]
    with pytest.raises(CaptureError) as error:
        servo_states_to_positions(states)
    assert "outside 0..1000" in str(error.value.issues)


def test_capture_buffer_enforces_order_and_bounds() -> None:
    buffer = CaptureBuffer(ARM_JOINTS, max_points=3)
    assert buffer.add("commanded", 0.0, [0.0] * 5)
    assert buffer.add("commanded", 0.02, [0.0] * 5)
    assert not buffer.add("commanded", 0.02, [0.0] * 5)
    assert buffer.out_of_order["commanded"] == 1
    assert buffer.add("commanded", 0.04, [0.0] * 5)
    assert not buffer.add("commanded", 0.06, [0.0] * 5)
    assert buffer.dropped["commanded"] == 1
    with pytest.raises(CaptureError):
        buffer.add("commanded", -1.0, [0.0] * 5)
    with pytest.raises(CaptureError):
        buffer.add("commanded", 0.1, [0.0] * 4)
    with pytest.raises(CaptureError):
        buffer.add("feedback", math.inf, [0.0] * 5)


def test_planned_loader_and_joint_selection() -> None:
    planned = load_planned_trajectory(
        {
            "kind": "moveit_planned_trajectory",
            "joint_names": ["r_joint", "joint2", "joint1"],
            "points": [
                {"t_s": 0.0, "positions": [0.0, 0.1, 0.2]},
                {"t_s": 1.0, "positions": [0.0, 0.3, 0.4], "velocities": [0.0, 0.0, 0.0]},
            ],
            "source": "unit-test",
        }
    )
    selected = select_planned_joints(planned, ["joint1", "joint2"])
    assert selected == [
        {"t_s": 0.0, "positions": [0.2, 0.1]},
        {"t_s": 1.0, "positions": [0.4, 0.3], "velocities": [0.0, 0.0]},
    ]


def test_planned_loader_rejects_bad_time_and_missing_joint() -> None:
    with pytest.raises(CaptureError):
        load_planned_trajectory(
            {
                "joint_names": ["joint1"],
                "points": [
                    {"t_s": 0.0, "positions": [0.0]},
                    {"t_s": 0.0, "positions": [0.1]},
                ],
            }
        )
    planned = load_planned_trajectory(
        {
            "joint_names": ["joint1"],
            "points": [
                {"t_s": 0.0, "positions": [0.0]},
                {"t_s": 1.0, "positions": [0.1]},
            ],
        }
    )
    with pytest.raises(CaptureError) as error:
        select_planned_joints(planned, ["joint1", "joint2"])
    assert "missing joint 'joint2'" in str(error.value.issues)


def test_static_planned_is_a_two_point_hold() -> None:
    planned = build_static_planned(ARM_JOINTS, [0.0] * 5, duration_s=2.0)
    assert planned["joint_names"] == list(ARM_JOINTS)
    assert [point["t_s"] for point in planned["points"]] == [0.0, 2.0]
    assert planned["metadata"]["motion"] is False


def _samples(start: float, count: int, values: list[float]) -> list[dict[str, object]]:
    return [
        {"t_s": start + index * 0.02, "positions": list(values)}
        for index in range(count)
    ]


def test_build_dataset_keeps_truth_and_offsets() -> None:
    planned = build_static_planned(ARM_JOINTS, [0.0] * 5, duration_s=1.0)
    dataset = build_dataset(
        name="unit capture",
        joint_names=list(ARM_JOINTS),
        planned=planned,
        commanded=_samples(0.02, 5, [0.0] * 5),
        feedback=_samples(0.03, 5, [0.001] * 5),
        commanded_source="command echo",
        feedback_source="servo states",
        planned_offset_s=0.01,
        metadata={"operator": "unit-test"},
    )
    assert dataset["schema_version"] == 1
    assert dataset["timebase"] == "shared_trajectory_start"
    assert dataset["planned"]["truth"] == "planned"
    assert dataset["commanded"]["truth"] == "commanded"
    assert dataset["feedback"]["truth"] == "measured"
    assert dataset["planned"]["points"][0]["t_s"] == pytest.approx(0.01)
    assert dataset["metadata"]["operator"] == "unit-test"


def test_build_dataset_rejects_short_or_unsorted_streams() -> None:
    planned = build_static_planned(ARM_JOINTS, [0.0] * 5, duration_s=1.0)
    with pytest.raises(CaptureError) as short:
        build_dataset(
            name="short",
            joint_names=list(ARM_JOINTS),
            planned=planned,
            commanded=_samples(0.0, 1, [0.0] * 5),
            feedback=_samples(0.0, 5, [0.0] * 5),
        )
    assert "2..20000" in str(short.value.issues)

    bad = _samples(0.0, 3, [0.0] * 5)
    bad[2]["t_s"] = 0.01
    with pytest.raises(CaptureError) as unsorted:
        build_dataset(
            name="unsorted",
            joint_names=list(ARM_JOINTS),
            planned=planned,
            commanded=bad,
            feedback=_samples(0.0, 5, [0.0] * 5),
        )
    assert "strictly increasing" in str(unsorted.value.issues)


def test_recorder_source_has_no_write_path() -> None:
    """The recorder must stay subscribe-only: no publishers, services or writes."""
    source = Path(__file__).resolve().parents[1] / "armpi_debug_gateway" / "recorder.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    calls = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "create_publisher" not in calls
    assert "create_client" not in calls
    assert "call_async" not in calls
    text = source.read_text(encoding="utf-8")
    for forbidden in (
        "SetBusServoState",
        "ServosPosition",
        "set_bus_servo",
        "follow_joint_trajectory",
    ):
        assert forbidden not in text


def test_dataset_from_capture_attaches_planned_stream_offline() -> None:
    buffer = CaptureBuffer(ARM_JOINTS)
    for index in range(5):
        t_s = index * 0.02
        buffer.add("commanded", t_s, [0.0] * 5)
        buffer.add("feedback", t_s + 0.001, [0.001] * 5)
    capture = buffer.capture_document(
        commanded_source="command echo",
        feedback_source="servo states",
        motion_start_s=0.0,
        notes=["unit-test"],
    )
    planned = build_static_planned(ARM_JOINTS, [0.0] * 5, duration_s=1.0)
    dataset = dataset_from_capture(capture, planned, name="offline attach")
    assert dataset["schema_version"] == 1
    assert len(dataset["feedback"]["points"]) == 5
    assert dataset["metadata"]["capture_notes"] == ["unit-test"]
    assert dataset["feedback"]["truth"] == "measured"

    with pytest.raises(CaptureError):
        dataset_from_capture({**capture, "kind": "wrong"}, planned)

