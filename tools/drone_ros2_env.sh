# Connects (or disconnects) this terminal to the Duckiedrone's ROS 2 (rmw_zenoh, domain 42).
#   source tools/drone_ros2_env.sh [host]   connect (default host: duckiedrone01.local)
#   source tools/drone_ros2_env.sh off      go back to the local middleware (CycloneDDS from .bashrc)
# Requires: sudo apt install ros-jazzy-rmw-zenoh-cpp
source /opt/ros/jazzy/setup.bash
if [ "$1" = "off" ]; then
  unset ZENOH_CONFIG_OVERRIDE ZENOH_ROUTER_CHECK_ATTEMPTS
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo "ROS 2 local: RMW=$RMW_IMPLEMENTATION, DOMAIN=$ROS_DOMAIN_ID"
  return 0 2>/dev/null || exit 0
fi
_DRONE_HOST=${1:-duckiedrone01.local}
if ! getent hosts "$_DRONE_HOST" >/dev/null; then
  echo "Cannot find $_DRONE_HOST. Is it powered on and on the same network (Ethernet or hotspot)?"
  unset _DRONE_HOST
  return 1 2>/dev/null || exit 1
fi
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
# No local router: the session connects directly to the drone's zenoh router (port 7447).
# The hostname is used instead of the IP so it works the same over Ethernet or hotspot.
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${_DRONE_HOST}:7447\"]"
echo "ROS 2 -> drone ${_DRONE_HOST} ($(getent hosts "$_DRONE_HOST" | awk '{print $1; exit}')), RMW=$RMW_IMPLEMENTATION, DOMAIN=$ROS_DOMAIN_ID"
echo "To go back to local ROS 2: source tools/drone_ros2_env.sh off"
unset _DRONE_HOST
