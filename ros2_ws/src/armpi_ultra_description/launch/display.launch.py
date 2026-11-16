"""Standalone ArmPi Ultra model display: robot_state_publisher + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share_dir = get_package_share_directory('armpi_ultra_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'armpi_ultra.xacro')
    rviz_config = os.path.join(share_dir, 'rviz', 'view.rviz')

    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_gui',
                default_value='true',
                description='Use joint_state_publisher_gui instead of joint_state_publisher',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Start RViz with the model view',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the simulation clock',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[robot_description, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
                condition=IfCondition(use_gui),
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                output='screen',
                condition=UnlessCondition(use_gui),
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
