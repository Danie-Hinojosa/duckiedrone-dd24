#!/bin/bash
# Descarga y compila PX4 SITL 1.15.4 (la versión del dron) en local/PX4-Autopilot,
# usando el Gazebo Harmonic que trae ros-jazzy-ros-gz. No necesita sudo.
# Uso: tools/sim/build_px4_sitl.sh
set -e
WS=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$WS/local" && touch "$WS/local/COLCON_IGNORE"
cd "$WS/local"
if [ ! -d PX4-Autopilot ]; then
  git clone --depth 1 --branch v1.15.4 --recurse-submodules --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git
fi
# El script de versión de PX4 busca etiquetas de NuttX que no vienen en un clon sin historial
git -C PX4-Autopilot/platforms/nuttx/NuttX/nuttx tag nuttx-11.0.0 2>/dev/null || true
if [ ! -d px4-venv ]; then
  python3 -m venv --system-site-packages px4-venv
  px4-venv/bin/pip install -q "empy==3.3.4" kconfiglib pyros-genmsg jsonschema future packaging \
      pyyaml jinja2 numpy toml psutil pyserial
fi
source px4-venv/bin/activate
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=$(ls -d /opt/ros/jazzy/opt/*/ | tr '\n' ':')/opt/ros/jazzy
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl -j"$(nproc)"
echo "Listo: $(pwd)/build/px4_sitl_default/bin/px4"
