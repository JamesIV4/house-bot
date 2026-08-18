"""Launch the same navigation interfaces with Nav2's official loopback base."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from house_bot_navigation.mock_map import ensure_mock_map


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("house_bot_navigation")
    params_file = LaunchConfiguration("params_file")
    destinations_file = LaunchConfiguration("destinations_file")
    map_file = LaunchConfiguration("map")
    ui_port = LaunchConfiguration("ui_port")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    mock_map = ensure_mock_map(os.path.join(tempfile.gettempdir(), "house_bot_mock_map"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "destinations_file",
                default_value=os.path.join(
                    package_share, "config", "destinations.yaml"
                ),
            ),
            DeclareLaunchArgument("map", default_value=str(mock_map)),
            DeclareLaunchArgument("ui_port", default_value="5000"),
            DeclareLaunchArgument("rosbridge_port", default_value="5001"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    params_file,
                    {"yaml_filename": map_file, "use_sim_time": True},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"autostart": True},
                    {"node_names": ["map_server"]},
                ],
            ),
            TimerAction(
                period=0.5,
                actions=[
                    Node(
                        package="nav2_loopback_sim",
                        executable="loopback_simulator",
                        name="loopback_simulator",
                        output="screen",
                        # The clock publisher itself must use wall time; setting
                        # it to simulated time creates a circular /clock wait.
                        parameters=[params_file, {"use_sim_time": False}],
                    )
                ],
            ),
            Node(
                package="house_bot_navigation",
                executable="mock_initial_pose",
                name="mock_initial_pose",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_scan_transform",
                output="screen",
                arguments=[
                    "--x",
                    "0.08",
                    "--y",
                    "0.0",
                    "--z",
                    "0.16",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "base_scan",
                ],
                parameters=[{"use_sim_time": False}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(package_share, "launch", "navigation.launch.py")
                ),
                launch_arguments={
                    "params_file": params_file,
                    "destinations_file": destinations_file,
                    "use_sim_time": "true",
                    "with_ui": "true",
                    "ui_port": ui_port,
                    "rosbridge_port": rosbridge_port,
                }.items(),
            ),
        ]
    )
