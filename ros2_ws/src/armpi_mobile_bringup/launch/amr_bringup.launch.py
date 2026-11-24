"""Full AMR bringup for ArmPi Ultra (draft).

Graph:
  LSLIDAR M10 -> /scan
  rtabmap_odom/icp_odometry -> /odom + TF odom -> base_link
  slam_toolbox (online async) -> TF map -> odom + /map
  Nav2 (MPPI Omni) -> /cmd_vel
  optional vendor mecanum node -> /ros_robot_controller/set_motor_speeds

SAFETY DEFAULTS
- start_base is false: Nav2 may compute and publish /cmd_vel, but nothing
  drives the motors until you explicitly set start_base:=true.
- The base has no hardware e-stop. Keep a physical power cutoff within reach
  and run the first tests with the wheels off the ground.
- The LiDAR pose arguments are relative to base_link and are PLACEHOLDERS;
  measure them before trusting any obstacle position.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('armpi_mobile_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    odom_params_file = LaunchConfiguration('odom_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    start_lidar = LaunchConfiguration('start_lidar')
    start_slam = LaunchConfiguration('start_slam')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_base = LaunchConfiguration('start_base')
    odom_source = LaunchConfiguration('odom_source')

    laser_port = LaunchConfiguration('laser_port')
    laser_frame = LaunchConfiguration('laser_frame')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_roll = LaunchConfiguration('laser_roll')

    declarations = [
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 params file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox params file'),
        DeclareLaunchArgument(
            'odom_params_file',
            default_value=os.path.join(pkg_share, 'config', 'icp_odometry.yaml'),
            description='RTAB-Map ICP odometry params file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_slam', default_value='true'),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument(
            'start_base', default_value='false',
            description='Start the vendor mecanum node (drives the motors!)'),
        DeclareLaunchArgument(
            'odom_source', default_value='icp',
            description="'icp' (rtabmap_odom) or 'open_loop' (bench only)"),
        DeclareLaunchArgument(
            'laser_port', default_value='/dev/lslidar_m10',
            description='Stable udev symlink for the M10 USB-serial adapter'),
        DeclareLaunchArgument('laser_frame', default_value='laser_link'),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(pkg_share, 'config', 'lslidar_m10.yaml'),
            description='LSLIDAR M10 driver params file'),
        # Relative to base_link (arm-mount root), not to the floor.
        DeclareLaunchArgument(
            'laser_x', default_value='0.12',
            description='LiDAR x relative to base_link (PROVISIONAL)'),
        DeclareLaunchArgument(
            'laser_y', default_value='0.0',
            description='LiDAR y relative to base_link (PROVISIONAL)'),
        DeclareLaunchArgument(
            'laser_z', default_value='0.03',
            description='LiDAR z relative to base_link (PROVISIONAL)'),
        DeclareLaunchArgument(
            'laser_yaw', default_value='0.0',
            description='LiDAR yaw relative to base_link (+x forward)'),
        DeclareLaunchArgument(
            'laser_pitch', default_value='0.0',
            description='LiDAR pitch relative to base_link'),
        DeclareLaunchArgument(
            'laser_roll', default_value='0.0',
            description='LiDAR roll relative to base_link'),
    ]

    # Humble positional form: x y z yaw pitch roll parent child.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        output='screen',
        arguments=[
            laser_x, laser_y, laser_z,
            laser_yaw, laser_pitch, laser_roll,
            'base_link', laser_frame,
        ],
        condition=IfCondition(start_lidar),
    )

    # lslidar_driver_node is a plain rclcpp Node, not a lifecycle node.
    # It starts publishing as soon as the serial port opens. The M10 motor is
    # treated as running at startup; publish 0 on /motor_control to stop it.
    lidar = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[
            lidar_params_file,
            {
                'serial_port': laser_port,
                'frame_id': laser_frame,
                'use_sim_time': use_sim_time,
            },
        ],
        condition=IfCondition(start_lidar),
    )

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[odom_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/scan'), ('odom', '/odom')],
        condition=IfCondition(
            PythonExpression(["'", odom_source, "' == 'icp'"])),
    )

    open_loop_odometry = Node(
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
        condition=IfCondition(
            PythonExpression(["'", odom_source, "' == 'open_loop'"])),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
        condition=IfCondition(start_slam),
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

    # The vendor mecanum node subscribes /cmd_vel and publishes the four
    # open-loop motor duties. It requires /ros_robot_controller to be running
    # (the vendor container normally provides it).
    base = Node(
        package='controller',
        executable='mecanum',
        name='mecanum_chassis_node',
        output='screen',
        condition=IfCondition(start_base),
    )

    return LaunchDescription(
        declarations + [
            laser_tf,
            lidar,
            icp_odometry,
            open_loop_odometry,
            slam,
            nav2,
            base,
        ])
