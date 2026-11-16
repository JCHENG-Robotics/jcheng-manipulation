"""Publish the ArmPi Ultra robot description and joint states (no RViz)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    display_launch = os.path.join(
        get_package_share_directory('armpi_ultra_description'),
        'launch',
        'display.launch.py',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the simulation clock',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(display_launch),
                launch_arguments={
                    'use_gui': 'false',
                    'use_rviz': 'false',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            )
        ]
    )
