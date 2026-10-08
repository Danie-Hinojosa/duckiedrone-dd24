# Conecta (o desconecta) esta terminal del ROS 2 del Duckiedrone (rmw_zenoh, dominio 42).
#   source tools/dron_ros2_env.sh [host]   conectar (host por defecto: duckiedrone01.local)
#   source tools/dron_ros2_env.sh off      volver al middleware local (CycloneDDS del .bashrc)
# Requiere: sudo apt install ros-jazzy-rmw-zenoh-cpp
source /opt/ros/jazzy/setup.bash
if [ "$1" = "off" ]; then
  unset ZENOH_CONFIG_OVERRIDE ZENOH_ROUTER_CHECK_ATTEMPTS
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo "ROS 2 local: RMW=$RMW_IMPLEMENTATION, DOMAIN=$ROS_DOMAIN_ID"
  return 0 2>/dev/null || exit 0
fi
_DRONE_HOST=${1:-duckiedrone01.local}
if ! getent hosts "$_DRONE_HOST" >/dev/null; then
  echo "No encuentro $_DRONE_HOST. ¿Está encendido y en la misma red (Ethernet o hotspot)?"
  unset _DRONE_HOST
  return 1 2>/dev/null || exit 1
fi
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=42
# Sin router local: la sesión se conecta directo al router zenoh del dron (puerto 7447).
# Se usa el nombre y no la IP para que funcione igual por Ethernet o por hotspot.
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${_DRONE_HOST}:7447\"]"
echo "ROS 2 -> dron ${_DRONE_HOST} ($(getent hosts "$_DRONE_HOST" | awk '{print $1; exit}')), RMW=$RMW_IMPLEMENTATION, DOMAIN=$ROS_DOMAIN_ID"
echo "Para volver al ROS 2 local: source tools/dron_ros2_env.sh off"
unset _DRONE_HOST
