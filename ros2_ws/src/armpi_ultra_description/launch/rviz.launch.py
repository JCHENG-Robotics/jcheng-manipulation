"""Launch only RViz with the ArmPi Ultra model view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('armpi_ultra_description'), 'rviz', 'view.rviz'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_config,
                description='Full path to the RViz config file',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rviz_config')],
            ),
        ]
    )
