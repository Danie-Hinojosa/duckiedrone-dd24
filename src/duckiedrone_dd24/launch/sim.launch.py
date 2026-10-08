"""Simulación del DD24 en la laptop: PX4 SITL 1.15.4 + Gazebo Harmonic + MAVROS.

    ros2 launch duckiedrone_dd24 sim.launch.py              # con la ventana de Gazebo
    ros2 launch duckiedrone_dd24 sim.launch.py gui:=false   # sin ventana
    ros2 launch duckiedrone_dd24 sim.launch.py rviz:=true

Con la simulación corriendo, los mismos nodos del dron funcionan en la laptop, por ejemplo:
    ros2 run duckiedrone_dd24 hover_test --height 0.4 --hold 5
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_px4(context):
    share = get_package_share_directory("duckiedrone_dd24")
    px4_dir = os.path.expanduser(LaunchConfiguration("px4_dir").perform(context))
    px4_bin = os.path.join(px4_dir, "build", "px4_sitl_default", "bin", "px4")
    if not os.path.exists(px4_bin):
        raise RuntimeError(f"No encuentro PX4 SITL en {px4_bin}. Compílalo con 'make px4_sitl' (ver README).")
    # PX4 busca los airframes en su carpeta de compilación: se copia el del DD24 en cada arranque
    airframes = os.path.join(px4_dir, "build", "px4_sitl_default", "etc", "init.d-posix", "airframes")
    shutil.copy(os.path.join(share, "sim", "airframes", "4061_gz_dd24"), airframes)

    models = os.path.join(share, "sim", "models")
    env = {
        "GZ_SIM_RESOURCE_PATH": models + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        "PX4_SYS_AUTOSTART": "4061",
        "PX4_GZ_MODEL_POSE": "0,0,0.06,0,0,0",
    }
    if LaunchConfiguration("gui").perform(context).lower() in ("false", "0"):
        env["HEADLESS"] = "1"
    return [ExecuteProcess(cmd=[px4_bin, "-d"], cwd=px4_dir, additional_env=env, output="screen",
                           name="px4_sitl")]


def start_mavros(context):
    from ament_index_python.packages import PackageNotFoundError
    from launch.actions import LogInfo
    try:
        get_package_share_directory("mavros")
    except PackageNotFoundError:
        return [LogInfo(msg="MAVROS no está instalado: la simulación corre, pero sin puente a ROS 2. "
                            "Instálalo con: sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras "
                            "&& sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh")]
    share = get_package_share_directory("duckiedrone_dd24")
    return [Node(package="mavros", executable="mavros_node", output="screen",
                 parameters=[os.path.join(share, "config", "mavros_sim.yaml")])]


def generate_launch_description():
    share = get_package_share_directory("duckiedrone_dd24")
    with open(os.path.join(share, "urdf", "dd24.urdf")) as f:
        urdf = f.read()
    vehicle = LaunchConfiguration("vehicle")
    return LaunchDescription([
        DeclareLaunchArgument("px4_dir", default_value="~/duckiedrone_ws/local/PX4-Autopilot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("vehicle", default_value="duckiedrone01"),
        # La simulación es local: no usar la sesión Zenoh del dron aunque la terminal la tenga
        SetEnvironmentVariable("RMW_IMPLEMENTATION", os.environ.get("DD24_LOCAL_RMW", "rmw_cyclonedds_cpp")),
        UnsetEnvironmentVariable("ZENOH_CONFIG_OVERRIDE"),

        OpaqueFunction(function=start_px4),

        # ToF simulado: Gazebo -> ROS 2 -> mismo tópico que el driver real
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge_tof",
             arguments=["/dd24/tof_bottom/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"]),
        Node(package="duckiedrone_dd24", executable="scan_to_range", parameters=[{"vehicle": vehicle}]),

        # MAVROS con la misma configuración de plugins que el dron (si está instalado)
        OpaqueFunction(function=start_mavros),

        # RViz opcional
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="duckiedrone_dd24", executable="drone_state_bridge",
             parameters=[{"demo": False, "vehicle": vehicle}], condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "dd24.rviz"), "-f", "map"],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
