#!/bin/bash
# Runs hover_test.py inside the drone's ros2-mavros container and copies the log back.
# Usage: ~/duckiedrone/control/run_hover.sh [hover_test.py args]
#   e.g.: run_hover.sh --dry-run
#         run_hover.sh --height 0.5 --hold 10 --hover 0.45
H=${DRONE:-duckie@duckiedrone01.local}
D=$(dirname "$(readlink -f "$0")")
LOG=/tmp/hover_test_$(date +%Y%m%d_%H%M%S).csv
scp -q "$D/../../src/duckiedrone_dd24/duckiedrone_dd24/hover_test.py" "$D/../../src/duckiedrone_dd24/duckiedrone_dd24/horizontal_control.py" $H:/tmp/ || exit 1
ssh -t $H "docker cp /tmp/hover_test.py ros2-mavros:/tmp/hover_test.py && docker cp /tmp/horizontal_control.py ros2-mavros:/tmp/horizontal_control.py && \
  docker exec -it ros2-mavros bash -lc 'source /opt/ros/*/setup.bash; python3 /tmp/hover_test.py --log $LOG $*'; \
  docker cp ros2-mavros:$LOG /tmp/ 2>/dev/null"
L="$D/../../logs/flights"; mkdir -p "$L" && scp -q $H:$LOG "$L/" 2>/dev/null && echo "Local log: $(readlink -f "$L")/$(basename $LOG)"
