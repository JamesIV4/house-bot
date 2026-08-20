"""Launch the hardware-independent House Bot Nav2 and browser UI layers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("house_bot_navigation")
    vizanti_share = get_package_share_directory("vizanti_server")

    params_file = LaunchConfiguration("params_file")
    base_calibration_file = LaunchConfiguration("base_calibration_file")
    destinations_file = LaunchConfiguration("destinations_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_ui = LaunchConfiguration("with_ui")
    ui_port = LaunchConfiguration("ui_port")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    ui_layout = LaunchConfiguration("ui_layout")

    common_parameters = [
        params_file,
        base_calibration_file,
        {"use_sim_time": use_sim_time},
    ]
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    navigation_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=common_parameters,
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ]
                },
            ],
        ),
        Node(
            package="house_bot_navigation",
            executable="named_goal_manager",
            name="named_goal_manager",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"destinations_file": destinations_file},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vizanti_share, "launch", "vizanti_server.launch.py")
            ),
            condition=IfCondition(with_ui),
            launch_arguments={
                "port": ui_port,
                "port_rosbridge": rosbridge_port,
                "default_widget_config": ui_layout,
            }.items(),
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "base_calibration_file",
                default_value=os.path.join(
                    package_share, "config", "base-calibration-placeholder.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "destinations_file",
                default_value=os.path.join(
                    package_share, "config", "destinations.yaml"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("with_ui", default_value="true"),
            DeclareLaunchArgument("ui_port", default_value="5000"),
            DeclareLaunchArgument("rosbridge_port", default_value="5001"),
            DeclareLaunchArgument(
                "ui_layout",
                default_value=os.path.join(package_share, "config", "vizanti_layout.json"),
            ),
            *navigation_nodes,
        ]
    )
