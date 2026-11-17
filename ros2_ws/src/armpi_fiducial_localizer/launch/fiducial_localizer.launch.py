"""Launch the ArmPi fiducial target localizer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CAMERA_FRAME = 'camera_optical_calibrated'
FRAME_CONFIG = os.path.join(
    get_package_share_directory('armpi_fiducial_localizer'),
    'config',
    'camera_optical_frame.yaml',
)


def generate_launch_description():
    """Build the launch description."""
    arguments = [
        DeclareLaunchArgument(
            'image_topic', default_value='image_raw'
        ),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='camera_info'
        ),
        DeclareLaunchArgument(
            'marker_dictionary', default_value='DICT_APRILTAG_36h11'
        ),
        DeclareLaunchArgument('marker_size_m', default_value='0.06'),
        DeclareLaunchArgument('marker_id', default_value='-1'),
        DeclareLaunchArgument(
            'camera_frame',
            default_value=CAMERA_FRAME,
            description=(
                'optical frame that owns the PnP pose; empty falls back to '
                'the CameraInfo body frame (legacy)'
            ),
        ),
        DeclareLaunchArgument(
            'output_frame',
            default_value='',
            description='empty: camera frame; e.g. base_link',
        ),
        DeclareLaunchArgument(
            'publish_optical_frame',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'publish rgb_camera_link -> camera_optical_calibrated '
                'from the measured frame contract'
            ),
        ),
        DeclareLaunchArgument(
            'frame_config',
            default_value=FRAME_CONFIG,
            description='measured camera frame contract YAML',
        ),
        DeclareLaunchArgument(
            'max_reprojection_error_px', default_value='3.0'
        ),
        DeclareLaunchArgument('tf_timeout_s', default_value='0.1'),
        DeclareLaunchArgument(
            'publish_on_tf_failure', default_value='true'
        ),
    ]
    frame_publisher = Node(
        package='armpi_fiducial_localizer',
        executable='camera_frame_publisher',
        name='armpi_camera_frame_publisher',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('frame_config')},
        ],
        condition=IfCondition(
            LaunchConfiguration('publish_optical_frame')),
    )
    localizer = Node(
        package='armpi_fiducial_localizer',
        executable='fiducial_localizer',
        name='armpi_fiducial_localizer',
        output='screen',
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration(
                    'camera_info_topic'
                ),
                'marker_dictionary': LaunchConfiguration(
                    'marker_dictionary'
                ),
                'marker_size_m': ParameterValue(
                    LaunchConfiguration('marker_size_m'),
                    value_type=float,
                ),
                'marker_id': ParameterValue(
                    LaunchConfiguration('marker_id'),
                    value_type=int,
                ),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'output_frame': LaunchConfiguration('output_frame'),
                'max_reprojection_error_px': ParameterValue(
                    LaunchConfiguration('max_reprojection_error_px'),
                    value_type=float,
                ),
                'tf_timeout_s': ParameterValue(
                    LaunchConfiguration('tf_timeout_s'),
                    value_type=float,
                ),
                'publish_on_tf_failure': ParameterValue(
                    LaunchConfiguration('publish_on_tf_failure'),
                    value_type=bool,
                ),
            }
        ],
    )
    return LaunchDescription(
        arguments + [frame_publisher, localizer])
