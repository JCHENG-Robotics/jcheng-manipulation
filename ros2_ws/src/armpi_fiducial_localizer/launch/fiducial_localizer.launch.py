"""Launch the ArmPi fiducial target localizer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            'output_frame',
            default_value='',
            description='empty: camera frame; e.g. base_link',
        ),
        DeclareLaunchArgument(
            'max_reprojection_error_px', default_value='3.0'
        ),
        DeclareLaunchArgument('tf_timeout_s', default_value='0.1'),
        DeclareLaunchArgument(
            'publish_on_tf_failure', default_value='true'
        ),
    ]
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
    return LaunchDescription(arguments + [localizer])
