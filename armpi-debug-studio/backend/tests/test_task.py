"""G8 task takeover tests: locate, fit, plan, verify, mock API."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient

from app.adapters.mock import MockRobotAdapter
from app.config import Settings
from app.domain.enums import Mode
from app.domain.grasp_fit import fit as grasp_fit
from app.domain.locate import (
    LocateParams,
    cube_box_present,
    cube_point,
    segment_floor_cube,
)
from app.domain.mapping import MappingConfig
from app.domain.models import MotionTarget
from app.domain.takeover import load_takeover_config
from app.domain.visual import compute_frames, load_urdf
from app.main import create_app
from app.persistence.store import EventStore
from app.service import RobotService

PROJECT_ROOT = Path(__file__).resolve().parents[2]
TAKEOVER_PATH = PROJECT_ROOT / "configs" / "takeover.yaml"
MAPPING_PATH = PROJECT_ROOT / "configs" / "joint_mapping.yaml"
URDF_PATH = (
    PROJECT_ROOT.parent
    / "ros2_ws"
    / "src"
    / "armpi_ultra_description"
    / "urdf"
    / "armpi_ultra.xacro"
)
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")


@pytest.fixture()
def takeover():
    return load_takeover_config(TAKEOVER_PATH)


@pytest.fixture()
def urdf():
    return load_urdf(URDF_PATH)


def _tcp(urdf):
    def tcp(joints):
        frames = compute_frames(
            urdf,
            {name: float(joints[index]) for index, name in enumerate(ARM_JOINTS)},
            {},
        )["frames"]
        for frame in frames:
            if frame["name"] == "end_effector_link":
                return np.asarray(frame["position_m"], dtype=np.float64)
        raise AssertionError("no end_effector_link")

    return tcp


def _floor_cloud(with_cube: bool = True) -> np.ndarray:
    x = np.linspace(0.10, 0.40, 260)
    y = np.linspace(-0.15, 0.15, 220)
    xx, yy = np.meshgrid(x, y)
    floor = np.column_stack(
        [xx.ravel(), yy.ravel(), np.full(xx.size, -0.080)]
    )
    if not with_cube:
        return floor
    side = np.linspace(0.21, 0.25, 40)
    cube_y = np.linspace(-0.020, 0.020, 40)
    cx, cy = np.meshgrid(side, cube_y)
    top = np.column_stack([cx.ravel(), cy.ravel(), np.full(cx.size, -0.040)])
    rng = np.random.default_rng(1)
    return np.vstack([floor + rng.normal(0, 0.0004, floor.shape), top])


def test_segment_synthetic_floor_cube() -> None:
    params = LocateParams(ransac_iterations=300, ransac_points=20000)
    report = segment_floor_cube(_floor_cloud(True), np.eye(4), params)
    assert report["success"] is True
    point, accepted = cube_point(report)
    assert accepted is True
    assert point is not None
    assert abs(point[0] - 0.23) < 0.004
    assert abs(point[1]) < 0.004
    assert abs(point[2] + 0.040) < 0.003
    assert report["segmentation"]["clusters"] >= 1

    empty = segment_floor_cube(_floor_cloud(False), np.eye(4), params)
    assert empty["success"] is False
    assert empty["segmentation"]["clusters"] == 0
    assert cube_box_present(empty, np.array([0.23, 0.0, -0.040])) is False


def test_cube_box_detects_the_floor_cube() -> None:
    report = segment_floor_cube(
        _floor_cloud(True),
        np.eye(4),
        LocateParams(ransac_iterations=300, ransac_points=20000),
    )
    assert cube_box_present(report, np.array([0.23, 0.0, -0.040])) is True
    assert cube_box_present(report, np.array([0.10, 0.0, -0.040])) is False


def test_fit_matches_the_recorded_2026_11_20_run(urdf) -> None:
    # cube_point + the validated constant correction from the 2026-11-20 trial.
    target = [
        0.22690956575501345 + 0.0137,
        -0.01390503513063539 + 0.0039,
        -0.05001748059159114 + 0.0089,
    ]
    result = grasp_fit(target, _tcp(urdf), j2=-0.6618, j3=-1.80117, j5=0.0)
    assert result["xy_error_mm"] < 0.1
    assert result["joints"][0] == pytest.approx(-0.0591, abs=0.002)
    assert result["joints"][3] == pytest.approx(-0.3303, abs=0.002)
    assert result["joints"][2] == pytest.approx(-1.80117, abs=1e-9)
    assert not result["reach_limited"]


def _pose_settings(tmp_path: Path) -> Settings:
    return Settings(
        mode=Mode.MOCK,
        robot_id="armpi-ultra-01",
        bind_host="127.0.0.1",
        port=8000,
        db_path=tmp_path / "task.db",
        mapping_path=MAPPING_PATH,
        frontend_dist=tmp_path / "dist",
        log_level="info",
        hardware_motion_enabled=False,
        motion_scope="pose",
        takeover_acknowledged=True,
        takeover_config_path=TAKEOVER_PATH,
    )


def _prepare_view(
    service: RobotService,
    takeover,
    clock,
    *,
    session: str = "task-test",
) -> dict:
    lease = service.acquire_lease(session, 600)
    view = takeover.preset("view_table").joints
    targets = [MotionTarget(name, view[name], 3.2) for name in ARM_JOINTS]
    preview = service.preview(session, targets)
    assert preview["valid"], preview
    service.execute(
        session_id=session,
        lease_id=lease["lease_id"],
        preview_token=preview["preview_token"],
        command_id=preview["command_id"],
        targets=targets,
        idempotency_key="prepare-view",
    )
    clock.advance(3.5)
    service.get_state()
    return lease


def _service(tmp_path: Path, mapping: MappingConfig, clock, takeover):
    settings = _pose_settings(tmp_path)
    store = EventStore(settings.db_path)
    adapter = MockRobotAdapter(mapping, clock=clock, takeover=takeover)
    return RobotService(
        settings, mapping, adapter, store, clock=clock, takeover=takeover
    )


def test_task_plan_requires_the_view(mapping, clock, takeover, tmp_path) -> None:
    service = _service(tmp_path, mapping, clock, takeover)
    try:
        plan = service.task_plan()
        assert plan["valid"] is False
        assert plan["errors"][0]["code"] == "TASK_NOT_AT_VIEW"
        assert plan["view_joints"]["joint2"] == pytest.approx(0.2574)
    finally:
        service.close()


def test_task_plan_and_verify_in_mock(mapping, clock, takeover, tmp_path) -> None:
    service = _service(tmp_path, mapping, clock, takeover)
    try:
        _prepare_view(service, takeover, clock)
        plan = service.task_plan()
        assert plan["valid"], plan
        assert plan["simulated"] is True
        assert len(plan["steps"]) == 13
        labels = [step["label"] for step in plan["steps"]]
        assert labels[:4] == ["open_gripper", "approach", "pre_grasp", "grasp"]
        assert "grip_verify" in labels
        assert "putback_grasp" in labels
        assert plan["fit"]["xy_error_mm"] < 1.0
        assert plan["grasp_joints"]["joint4"] == pytest.approx(
            plan["fit"]["joints"][3], abs=1e-6
        )
        verify = service.task_verify(plan["plan_id"])
        assert verify["held"] is True
        assert verify["simulated"] is True
        # The plan-time validation already checks every step from the previous
        # target. A preview from the *current* view would correctly reject the
        # later steps (the arm has not moved there yet), so it is not a valid
        # test of the plan's internal consistency.
    finally:
        service.close()


def test_task_api_plan_and_verify(tmp_path, mapping, clock, takeover) -> None:
    settings = _pose_settings(tmp_path)
    app = create_app(settings=settings, clock=clock)
    with TestClient(app) as client:
        lease = client.post(
            "/api/control/acquire",
            json={"session_id": "api-task", "ttl_s": 600},
        ).json()
        view = takeover.preset("view_table").joints
        targets = [
            {"joint": name, "target_rad": view[name], "duration_s": 3.2}
            for name in ARM_JOINTS
        ]
        preview = client.post(
            "/api/motion/preview",
            json={"session_id": "api-task", "targets": targets},
        ).json()
        assert preview["valid"], preview
        execute = client.post(
            "/api/motion/execute",
            headers={"Idempotency-Key": "api-view"},
            json={
                "session_id": "api-task",
                "lease_id": lease["lease_id"],
                "preview_token": preview["preview_token"],
                "command_id": preview["command_id"],
                "targets": targets,
            },
        ).json()
        assert execute["status"] == "Dispatched"
        clock.advance(3.5)
        assert client.get("/api/state").status_code == 200

        plan = client.post("/api/task/plan").json()
        assert plan["valid"] is True, plan
        assert plan["plan_id"].startswith("task_")
        verify = client.post(
            "/api/task/verify", json={"plan_id": plan["plan_id"]}
        ).json()
        assert verify["held"] is True
        stale = client.post(
            "/api/task/verify", json={"plan_id": "task_other"}
        )
        assert stale.status_code == 409
        assert stale.json()["error"]["code"] == "TASK_PLAN_STALE"


def _clipped_cloud(center_x: float = 0.231, cutoff_m: float = 0.240) -> np.ndarray:
    """A floor cube whose top near edge is inside the sensor's min range."""
    x = np.linspace(0.10, 0.40, 260)
    y = np.linspace(-0.15, 0.15, 220)
    xx, yy = np.meshgrid(x, y)
    floor = np.column_stack([xx.ravel(), yy.ravel(), np.full(xx.size, -0.080)])
    side = np.linspace(center_x - 0.020, center_x + 0.020, 40)
    cube_y = np.linspace(-0.0197, 0.0197, 40)
    cx, cy = np.meshgrid(side, cube_y)
    top = np.column_stack([cx.ravel(), cy.ravel(), np.full(cx.size, -0.040)])
    cloud = np.vstack([floor, top])
    ranges = np.linalg.norm(cloud, axis=1)
    return cloud[ranges >= cutoff_m]


def test_segment_recovers_min_range_clipped_top() -> None:
    cloud = _clipped_cloud()
    params = LocateParams(
        ransac_iterations=300,
        ransac_points=20000,
        cube_size_m=0.0394,
    )
    report = segment_floor_cube(cloud, np.eye(4), params)
    selected = report["selected"]
    assert "rejection" not in report
    assert selected["top_recovered"]["reason"] == "min_range_clip"
    assert selected["top_recovered"]["clipped_side"] == "a_min"
    point, accepted = cube_point(report)
    assert accepted is True
    assert point is not None
    assert abs(point[0] - 0.2313) < 0.0015
    assert abs(point[1]) < 0.0015
    assert abs(point[2] + 0.040) < 0.003
    assert selected["top_recovered"]["centroid_bias_mm"] > 3.0


def test_segment_keeps_rejecting_clipped_top_without_recovery() -> None:
    cloud = _clipped_cloud()
    params = LocateParams(ransac_iterations=300, ransac_points=20000)
    report = segment_floor_cube(cloud, np.eye(4), params)
    assert report["success"] is False
    assert report["rejection"]["reason"] == "partial_top_view"
    _, accepted = cube_point(report)
    assert accepted is False


def test_takeover_task_enables_the_clip_recovery(takeover) -> None:
    assert takeover.task is not None
    assert takeover.task.locate.cube_size_m == pytest.approx(0.0394)
    assert takeover.task.locate.recovery_intact_extent_m == pytest.approx(0.030)
