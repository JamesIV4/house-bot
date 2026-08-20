"""Launch the calibrated, initially disarmed real House Bot base bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("house_bot_navigation")
    calibration_file = LaunchConfiguration("calibration_file")
    pi_host = LaunchConfiguration("pi_host")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(
                    package_share, "config", "base-calibration-placeholder.yaml"
                ),
            ),
            DeclareLaunchArgument("pi_host", default_value="192.168.0.241"),
            Node(
                package="house_bot_navigation",
                executable="base_driver",
                name="house_bot_base",
                output="screen",
                parameters=[calibration_file, {"pi_host": pi_host}],
            ),
        ]
    )
