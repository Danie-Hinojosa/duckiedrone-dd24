#!/bin/bash
# Corre hover_test.py dentro del contenedor ros2-mavros del dron y trae el log.
# Uso: ~/duckiedrone/control/run_hover.sh [args de hover_test.py]
#   ej: run_hover.sh --dry-run
#       run_hover.sh --height 0.5 --hold 10 --hover 0.45
H=${DRONE:-duckie@duckiedrone01.local}
D=$(dirname "$(readlink -f "$0")")
LOG=/tmp/hover_test_$(date +%Y%m%d_%H%M%S).csv
scp -q "$D/../../src/duckiedrone_dd24/duckiedrone_dd24/hover_test.py" $H:/tmp/hover_test.py || exit 1
ssh -t $H "docker cp /tmp/hover_test.py ros2-mavros:/tmp/hover_test.py && \
  docker exec -it ros2-mavros bash -lc 'source /opt/ros/*/setup.bash; python3 /tmp/hover_test.py --log $LOG $*'; \
  docker cp ros2-mavros:$LOG /tmp/ 2>/dev/null"
L="$D/../../logs/flights"; mkdir -p "$L" && scp -q $H:$LOG "$L/" 2>/dev/null && echo "Log local: $(readlink -f "$L")/$(basename $LOG)"
