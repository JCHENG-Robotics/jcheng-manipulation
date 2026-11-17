"""
Camera frame contract helpers (no ROS runtime dependency).

The driver's ``rgb_camera_link`` is a body frame; OpenCV PnP returns the
optical frame. These helpers keep the frame name, the pose-frame resolution
and the measured body -> optical contract in one place.
"""

import os

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - source checkout without ROS
    get_package_share_directory = None

import yaml

CAMERA_OPTICAL_FRAME = 'camera_optical_calibrated'
CONFIG_NAME = 'camera_optical_frame.yaml'


def default_config_path():
    """Return the installed frame contract, or the source-tree copy."""
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory('armpi_fiducial_localizer')
            return os.path.join(share, 'config', CONFIG_NAME)
        except Exception:  # noqa: BLE001 - running from a source checkout
            pass
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config',
        CONFIG_NAME,
    )


def load_frame_config(path):
    """Load and validate a frame-contract YAML file."""
    with open(path, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    for key in (
        'parent_frame',
        'child_frame',
        'translation_m',
        'quaternion_xyzw',
    ):
        if key not in config:
            raise ValueError(
                'frame contract {} is missing {}'.format(path, key))
    if len(config['translation_m']) != 3:
        raise ValueError('translation_m must have 3 values')
    if len(config['quaternion_xyzw']) != 4:
        raise ValueError('quaternion_xyzw must have 4 values')
    norm = sum(value * value for value in config['quaternion_xyzw']) ** 0.5
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(
            'quaternion is not normalised (norm={:.6f})'.format(norm))
    config['translation_m'] = [
        float(value) for value in config['translation_m']
    ]
    config['quaternion_xyzw'] = [
        float(value) for value in config['quaternion_xyzw']
    ]
    return config


def resolve_pose_frame(camera_frame, message_frame, camera_info_frame):
    """
    Return the frame that owns a PnP pose.

    ``camera_frame`` is the explicit optical frame and wins when set. The
    fallbacks preserve the old behaviour for callers that deliberately leave
    it empty.
    """
    return (
        camera_frame
        or message_frame
        or camera_info_frame
        or CAMERA_OPTICAL_FRAME
    )
