"""Ver el dron real en RViz desde la laptop.

Antes, en la misma terminal:  source tools/dron_ros2_env.sh   (rmw_zenoh, dominio 42)
Luego:                         ros2 launch duckiedrone_dd24 drone_rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("duckiedrone_dd24")
    with open(os.path.join(share, "urdf", "dd24.urdf")) as f:
        urdf = f.read()
    vehicle = LaunchConfiguration("vehicle")
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="duckiedrone01"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="duckiedrone_dd24", executable="drone_state_bridge",
             parameters=[{"demo": False, "vehicle": vehicle}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "dd24.rviz"),
                                                             "-f", "map"]),
    ])
