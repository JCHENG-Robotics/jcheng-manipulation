"""Bench-only Nav2 bringup for the ArmPi AMR draft.

Starts the provisional open-loop odometry and Nav2 (MPPI Omni) with NO LiDAR
and NO chassis controller. Use it with the wheels off the ground to exercise
the Nav2 lifecycle, TF graph, costmaps and cmd_vel output before the LiDAR
and laser odometry are installed.

This launch never starts the mecanum node, so it cannot move the base by
itself. It also does not publish a laser TF or a /scan topic.

For the real stack see amr_bringup.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('armpi_mobile_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start_odom = LaunchConfiguration('start_open_loop_odom')
    start_nav2 = LaunchConfiguration('start_nav2')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Nav2 params file')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically configure/activate Nav2 lifecycle nodes')
    declare_start_odom = DeclareLaunchArgument(
        'start_open_loop_odom', default_value='true',
        description='Start the provisional open-loop /cmd_vel odometry')
    declare_start_nav2 = DeclareLaunchArgument(
        'start_nav2', default_value='true',
        description='Start Nav2 navigation servers')

    open_loop_odom = Node(
        package='armpi_mobile_bringup',
        executable='open_loop_odom',
        name='open_loop_odom',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
            'rate_hz': 30.0,
            'cmd_timeout': 0.5,
        }],
        condition=IfCondition(start_odom),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items(),
        condition=IfCondition(start_nav2),
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        declare_start_odom,
        declare_start_nav2,
        open_loop_odom,
        nav2,
    ])
