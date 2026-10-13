"""Hover test with our own altitude controller. Runs ON THE DRONE (container with MAVROS):
    ros2 launch duckiedrone_dd24 hover.launch.py height:=1.0 hold:=20.0
All other settings come from config/hover.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("duckiedrone_dd24"), "config", "hover.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("height", default_value="0.4"),
        DeclareLaunchArgument("hold", default_value="5.0"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        Node(package="duckiedrone_dd24", executable="hover_test", name="dd_hover_test", output="screen",
             emulate_tty=True,
             parameters=[cfg, {"height": LaunchConfiguration("height"),
                               "hold": LaunchConfiguration("hold"),
                               "dry_run": LaunchConfiguration("dry_run")}]),
    ])
