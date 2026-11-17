"""Launch the conservative ArmPi safety supervisor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the safety launch description."""
    config = os.path.join(
        get_package_share_directory('armpi_safety'),
        'config',
        'safety_limits.yaml',
    )
    arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=config,
            description='provisional safety limits YAML',
        ),
        DeclareLaunchArgument(
            'joint_state_topic', default_value='/joint_states'
        ),
        DeclareLaunchArgument('publish_rate_hz', default_value='5.0'),
        DeclareLaunchArgument(
            'workspace_tip_link', default_value='end_effector_link'
        ),
    ]
    supervisor = Node(
        package='armpi_safety',
        executable='safety_supervisor',
        name='armpi_safety',
        output='screen',
        parameters=[
            {
                'config_file': LaunchConfiguration('config_file'),
                'joint_state_topic': LaunchConfiguration(
                    'joint_state_topic'
                ),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_rate_hz'),
                    value_type=float,
                ),
                'workspace_tip_link': LaunchConfiguration(
                    'workspace_tip_link'
                ),
            }
        ],
    )
    return LaunchDescription(arguments + [supervisor])
