"""Ver el modelo del DD24 en RViz sin el dron: ros2 launch duckiedrone_dd24 display.launch.py"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, UnsetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("duckiedrone_dd24")
    with open(os.path.join(share, "urdf", "dd24.urdf")) as f:
        urdf = f.read()
    return LaunchDescription([
        # Solo es visualización local: no usar la sesión Zenoh del dron aunque la terminal la tenga cargada.
        SetEnvironmentVariable("RMW_IMPLEMENTATION", os.environ.get("DD24_LOCAL_RMW", "rmw_cyclonedds_cpp")),
        UnsetEnvironmentVariable("ZENOH_CONFIG_OVERRIDE"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        Node(package="duckiedrone_dd24", executable="drone_state_bridge", parameters=[{"demo": True}]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "dd24.rviz"),
                                                             "-f", "base_link"]),
    ])
