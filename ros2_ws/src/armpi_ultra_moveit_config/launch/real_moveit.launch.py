"""MoveIt 2 for the real ArmPi robot: move_group + RViz only.

The robot (Humble) provides /joint_states, /tf and the
/arm_controller/follow_joint_trajectory action server. This launch deliberately
does not start mock controllers, robot_state_publisher, joint_state_publisher or
a static transform publisher: those would duplicate the robot's topics and
action names and make MoveIt execute the same goal on two controllers.

It is intended to run in a Humble container on the laptop while the robot runs
the Humble bringup stack.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Build the real-robot MoveIt launch description."""
    moveit_config = (
        MoveItConfigsBuilder(
            "armpi_ultra", package_name="armpi_ultra_moveit_config"
        )
        # Bind these files explicitly: the real launch must use the SRDF named
        # states and the FollowJointTrajectory mapping from this package, not
        # whatever a stale install or another MoveIt config might provide.
        .robot_description_semantic(file_path="config/armpi_ultra.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt config",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    get_package_share_directory(
                        "armpi_ultra_moveit_config"
                    ),
                    "rviz",
                    "moveit.rviz",
                ),
                description="RViz configuration file",
            ),
            move_group,
            rviz,
        ]
    )
