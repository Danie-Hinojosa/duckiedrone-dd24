"""DD24 simulation on the laptop: PX4 SITL 1.15.4 + Gazebo Harmonic + MAVROS.

    ros2 launch duckiedrone_dd24 sim.launch.py              # with the Gazebo window
    ros2 launch duckiedrone_dd24 sim.launch.py gui:=false   # no window
    ros2 launch duckiedrone_dd24 sim.launch.py rviz:=true

With the simulation running, the same drone nodes work on the laptop, for example:
    ros2 run duckiedrone_dd24 hover_test --height 0.4 --hold 5
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction, UnsetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_px4(context):
    share = get_package_share_directory("duckiedrone_dd24")
    px4_dir = os.path.expanduser(LaunchConfiguration("px4_dir").perform(context))
    px4_bin = os.path.join(px4_dir, "build", "px4_sitl_default", "bin", "px4")
    if not os.path.exists(px4_bin):
        raise RuntimeError(f"PX4 SITL not found at {px4_bin}. Build it with 'make px4_sitl' (see README).")
    # PX4 looks for airframes in its build folder, so the DD24 one is copied there on every start
    airframes = os.path.join(px4_dir, "build", "px4_sitl_default", "etc", "init.d-posix", "airframes")
    shutil.copy(os.path.join(share, "sim", "airframes", "4061_gz_dd24"), airframes)

    models = os.path.join(share, "sim", "models")
    world = os.path.join(share, "sim", "worlds", LaunchConfiguration("world").perform(context) + ".sdf")
    resource = models + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    headless = LaunchConfiguration("gui").perform(context).lower() in ("false", "0")
    # Gazebo first, with our world; PX4 detects the running world and connects to it
    gz_cmd = ["gz", "sim", "-r", "--verbose=1"] + (["-s"] if headless else []) + [world]
    gz = ExecuteProcess(cmd=gz_cmd, additional_env={"GZ_SIM_RESOURCE_PATH": resource}, output="screen",
                        name="gazebo")
    env = {
        "GZ_SIM_RESOURCE_PATH": resource,
        "PX4_SYS_AUTOSTART": "4061",
        "PX4_GZ_MODEL_POSE": "0,0,0.06,0,0,0",
        "HEADLESS": "1",  # the window (if requested) is already opened by the Gazebo process above
    }
    px4 = ExecuteProcess(cmd=[px4_bin, "-d"], cwd=px4_dir, additional_env=env, output="screen", name="px4_sitl")
    return [gz, TimerAction(period=5.0, actions=[px4])]


def start_mavros(context):
    from ament_index_python.packages import PackageNotFoundError
    from launch.actions import LogInfo
    try:
        get_package_share_directory("mavros")
    except PackageNotFoundError:
        return [LogInfo(msg="MAVROS is not installed: the simulation runs, but with no bridge to ROS 2. "
                            "Install it with: sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras "
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
        DeclareLaunchArgument("world", default_value="dd24_lab", description="world in sim/worlds"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("vehicle", default_value="duckiedrone01"),
        # The simulation is local: don't use the drone's Zenoh session even if the terminal has it
        SetEnvironmentVariable("RMW_IMPLEMENTATION", os.environ.get("DD24_LOCAL_RMW", "rmw_cyclonedds_cpp")),
        UnsetEnvironmentVariable("ZENOH_CONFIG_OVERRIDE"),

        OpaqueFunction(function=start_px4),

        # Simulated ToF: Gazebo -> ROS 2 -> same topic as the real driver
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge_tof",
             arguments=["/dd24/tof_bottom/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"]),
        Node(package="duckiedrone_dd24", executable="scan_to_range", parameters=[{"vehicle": vehicle}]),

        # Simulated camera -> ROS 2
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge_camera",
             arguments=["/dd24/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        "/dd24/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        # ground truth pose (to validate estimators and controllers)
                        "/dd24/sim/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry"]),
        # AprilTag positioning (same grid and size as the A4 PDF)
        Node(package="duckiedrone_dd24", executable="tag_localizer", output="screen",
             # In Gazebo the tag texture ends up rotated 90° from the printed sheet (measured against
             # ground truth), so the tag yaw is pi here and -pi/2 on the real drone.
             parameters=[{"tag_size": 0.16, "publish_debug": True, "tag_yaw_offset": 3.14159265}]),

        # MAVROS with the same plugin config as the drone (if installed)
        OpaqueFunction(function=start_mavros),

        # Optional RViz
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="duckiedrone_dd24", executable="drone_state_bridge",
             parameters=[{"demo": False, "vehicle": vehicle}], condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="rviz2", executable="rviz2",
             arguments=["-d", os.path.join(share, "rviz", "dd24.rviz"), "-f", "map"],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
