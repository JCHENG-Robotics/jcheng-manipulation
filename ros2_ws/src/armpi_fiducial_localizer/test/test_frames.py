"""Tests for the camera frame contract and pose-frame resolution."""

import math

from armpi_fiducial_localizer.frames import (
    CAMERA_OPTICAL_FRAME,
    default_config_path,
    load_frame_config,
    resolve_pose_frame,
)

import pytest
import yaml


def test_contract_names_and_values():
    """The measured contract has the expected frames and quaternion."""
    config = load_frame_config(default_config_path())
    assert config['parent_frame'] == 'rgb_camera_link'
    assert config['child_frame'] == CAMERA_OPTICAL_FRAME
    translation = config['translation_m']
    assert translation == pytest.approx(
        [0.005408, 0.010058, -0.027380], abs=1e-6)
    quaternion = config['quaternion_xyzw']
    assert math.sqrt(sum(value * value for value in quaternion)) == \
        pytest.approx(1.0, abs=1e-9)
    angle_deg = math.degrees(2.0 * math.acos(abs(quaternion[3])))
    assert angle_deg == pytest.approx(89.798, abs=0.001)


def test_resolve_pose_frame_prefers_explicit_camera_frame():
    """An explicit optical frame wins over the CameraInfo body frame."""
    assert resolve_pose_frame(
        CAMERA_OPTICAL_FRAME, 'rgb_camera_link', 'rgb_camera_link'
    ) == CAMERA_OPTICAL_FRAME


def test_resolve_pose_frame_legacy_fallbacks():
    """Empty camera_frame preserves the old message/info fallback."""
    assert resolve_pose_frame(
        '', 'rgb_camera_link', 'depth_camera_link'
    ) == 'rgb_camera_link'
    assert resolve_pose_frame(
        '', '', 'depth_camera_link'
    ) == 'depth_camera_link'
    assert resolve_pose_frame('', '', '') == CAMERA_OPTICAL_FRAME


def test_unnormalised_quaternion_is_rejected(tmp_path):
    """A malformed contract is rejected instead of published."""
    path = tmp_path / 'bad.yaml'
    path.write_text(
        yaml.safe_dump(
            {
                'parent_frame': 'rgb_camera_link',
                'child_frame': CAMERA_OPTICAL_FRAME,
                'translation_m': [0.0, 0.0, 0.0],
                'quaternion_xyzw': [0.0, 0.0, 0.0, 2.0],
            }
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='not normalised'):
        load_frame_config(str(path))


def test_missing_field_is_rejected(tmp_path):
    """A contract without a child frame is rejected."""
    path = tmp_path / 'bad.yaml'
    path.write_text(
        yaml.safe_dump(
            {
                'parent_frame': 'rgb_camera_link',
                'translation_m': [0.0, 0.0, 0.0],
                'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            }
        ),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='child_frame'):
        load_frame_config(str(path))
