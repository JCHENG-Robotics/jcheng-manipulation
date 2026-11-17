"""
Mock-only visual grasp chain.

Starts MoveIt mock controllers, a down-looking camera frame, a synthetic
AprilTag, the fiducial localizer, and the safety supervisor. The one-shot
bridge is run separately so its exit code and JSON report are explicit:

    ros2 launch armpi_grasp_planner mock_visual_grasp.launch.py
    ros2 run armpi_grasp_planner grasp_target_bridge \
      --safety-service /armpi_safety/validate_trajectory \
      --json-out /tmp/bridge.json

NEVER run this launch while the real ArmPi stack is up: it starts mock
controllers and a second /joint_states and /tf source on the same DDS domain.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TARGET_X = 0.1382
TARGET_Y = 0.0225
TARGET_Z = 0.0896
MARKER_DISTANCE = 0.26
CAMERA_Z = TARGET_Z + MARKER_DISTANCE


def generate_launch_description():
    """Build the mock visual-grasp launch description."""
    moveit_launch = os.path.join(
        get_package_share_directory('armpi_ultra_moveit_config'),
        'launch',
        'demo.launch.py',
    )
    localizer_launch = os.path.join(
        get_package_share_directory('armpi_fiducial_localizer'),
        'launch',
        'fiducial_localizer.launch.py',
    )
    safety_launch = os.path.join(
        get_package_share_directory('armpi_safety'),
        'launch',
        'safety_supervisor.launch.py',
    )
    arguments = [
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='start RViz with the mock robot',
        ),
        DeclareLaunchArgument(
            'start_moveit', default_value='true',
            description='start MoveIt mock and controllers',
        ),
        DeclareLaunchArgument(
            'start_safety', default_value='true',
            description='start the safety supervisor',
        ),
        DeclareLaunchArgument(
            'domain_id',
            default_value='77',
            description='isolated DDS domain for the mock chain',
        ),
        DeclareLaunchArgument(
            'localhost_only',
            default_value='true',
            description='keep the mock DDS graph on this host',
        ),
        DeclareLaunchArgument(
            'camera_x', default_value=str(TARGET_X)
        ),
        DeclareLaunchArgument(
            'camera_y', default_value=str(TARGET_Y)
        ),
        DeclareLaunchArgument(
            'camera_z', default_value=str(CAMERA_Z)
        ),
        DeclareLaunchArgument(
            'camera_roll',
            default_value=str(math.pi),
            description='camera optical +z points down (180 deg roll)',
        ),
        DeclareLaunchArgument(
            'marker_distance', default_value=str(MARKER_DISTANCE)
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.06'),
    ]

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'ros2_control_hardware_type': 'mock_components',
            'use_sim_time': 'false',
            'use_gazebo': 'false',
            'use_real': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_moveit')),
    )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mock_camera_optical_tf',
        output='log',
        arguments=[
            LaunchConfiguration('camera_x'),
            LaunchConfiguration('camera_y'),
            LaunchConfiguration('camera_z'),
            '0.0',
            '0.0',
            LaunchConfiguration('camera_roll'),
            'base_link',
            'camera_optical_calibrated',
        ],
    )
    synthetic_marker = Node(
        package='armpi_fiducial_localizer',
        executable='synthetic_marker_publisher',
        name='synthetic_marker_publisher',
        output='screen',
        arguments=[
            '--frame-id',
            'camera_optical_calibrated',
            '--tvec',
            '0.0',
            '0.0',
            LaunchConfiguration('marker_distance'),
            '--rvec',
            str(math.pi),
            '0.0',
            '0.0',
            '--marker-id',
            LaunchConfiguration('marker_id'),
            '--marker-size',
            LaunchConfiguration('marker_size_m'),
            '--width',
            '640',
            '--height',
            '480',
            '--fx',
            '700',
            '--fy',
            '700',
        ],
    )
    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localizer_launch),
        launch_arguments={
            'publish_optical_frame': 'false',
            'camera_frame': 'camera_optical_calibrated',
            'output_frame': '',
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'tf_timeout_s': '0.2',
        }.items(),
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(safety_launch),
        condition=IfCondition(LaunchConfiguration('start_safety')),
    )
    return LaunchDescription(
        arguments
        + [
            SetEnvironmentVariable(
                'ROS_DOMAIN_ID', LaunchConfiguration('domain_id')
            ),
            SetEnvironmentVariable(
                'ROS_LOCALHOST_ONLY',
                LaunchConfiguration('localhost_only'),
            ),
        ]
        + [moveit, camera_tf, synthetic_marker, localizer, safety]
    )
