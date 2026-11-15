"""Gazebo bring-up with a validated controller-config selection and fail-closed chain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

from arm_description.controller_config import ControllerConfigError, resolve_controller_config


CONTROLLER_MANAGER = "/controller_manager"
STAGE_TIMEOUT_SECONDS = 60.0


def _shutdown(stage, reason):
    return [
        LogInfo(msg=f"Gate 2 launch failure: stage={stage}; {reason}"),
        EmitEvent(event=Shutdown(reason=f"{stage}: {reason}")),
    ]


def _start_stage(stage, action):
    """Start a stage and its deadline in the same launch event."""
    return [action, _watchdog(stage)]


def _after_success(stage, next_stage=None):
    def callback(event, context):
        if event.returncode == 0:
            context.launch_configurations[f"gate2_{stage}_completed"] = "true"
            return _start_stage(*next_stage) if next_stage else []
        return _shutdown(stage, f"command={event.process_name}; returncode={event.returncode}")
    return callback


def _watchdog(stage):
    def callback(context):
        if context.launch_configurations.get(f"gate2_{stage}_completed") == "true":
            return []
        return _shutdown(stage, f"wall deadline exceeded ({STAGE_TIMEOUT_SECONDS}s)")
    return TimerAction(period=STAGE_TIMEOUT_SECONDS, actions=[OpaqueFunction(function=callback)])


def _launch_setup(context, *unused):
    pkg_share = get_package_share_directory("arm_description")
    profile = LaunchConfiguration("controller_profile").perform(context)
    requested_config = LaunchConfiguration("controller_config").perform(context)
    try:
        selection = resolve_controller_config(pkg_share, profile, requested_config)
    except (ControllerConfigError, OSError) as exc:
        raise RuntimeError(f"Gate 2 launch failure: stage=controller_config; {exc}") from exc

    xacro_file = os.path.join(pkg_share, "urdf", "simple_arm.xacro")
    try:
        robot_description_raw = xacro.process_file(
            xacro_file, mappings={"controller_config": str(selection.realpath)}).toxml()
    except Exception as exc:
        raise RuntimeError(f"Gate 2 launch failure: stage=xacro; {exc}") from exc

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": ParameterValue(robot_description_raw, value_type=str),
                     "use_sim_time": True}],
    )
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": " -r -v 4 empty.sdf", "on_exit_shutdown": "true"}.items())
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"], output="screen")
    spawn = Node(package="ros_gz_sim", executable="create",
                 arguments=["-topic", "robot_description", "-name", "simple_arm"], output="screen")
    common_spawner_args = ["-c", CONTROLLER_MANAGER, "--controller-manager-timeout", "55",
                           "--switch-timeout", "55", "--service-call-timeout", "55"]
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", *common_spawner_args], output="screen")
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["arm_controller", *common_spawner_args], output="screen")

    # Register handlers before their target actions. Only a zero exit proceeds.
    return [
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=_after_success("spawn", ("jsb", jsb)))),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=_after_success("jsb", ("arm", arm)))),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=_after_success("arm", None))),
        RegisterEventHandler(OnProcessExit(target_action=robot_state_publisher,
                                            on_exit=lambda event, context: _shutdown("robot_state_publisher", f"returncode={event.returncode}"))),
        RegisterEventHandler(OnProcessExit(target_action=clock_bridge,
                                            on_exit=lambda event, context: _shutdown("clock_bridge", f"returncode={event.returncode}"))),
        _watchdog("spawn"),
        gazebo, clock_bridge, robot_state_publisher, spawn,
        LogInfo(msg=("controller config selected: "
                     f"profile={selection.profile} input={selection.original_input!r} "
                     f"share={selection.share_directory} logical={selection.logical_path} "
                     f"realpath={selection.realpath} sha256={selection.sha256}")),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("controller_profile", default_value="default",
                              description="One of: default, gain_0p1, gain_0p2."),
        DeclareLaunchArgument("controller_config", default_value="",
                              description="Optional absolute or package-share/config-relative YAML path."),
        OpaqueFunction(function=_launch_setup),
    ])
