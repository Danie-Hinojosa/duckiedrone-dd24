#!/bin/bash
# Reapplies the ToF->PX4 workaround (dt-ros2-interface bug). It is lost when ros2-mavros or the drone restarts.
# Usage: ~/duckiedrone/workaround/apply.sh   (asks for the duckie password if there is no sshpass/key)
set -e
H=duckie@duckiedrone01.local
D=$(dirname "$0")
scp -q "$D/ds.py" "$D/relay.py" $H:/tmp/
ssh $H 'docker cp /tmp/ds.py ros2-mavros:/tmp/ds.py && docker cp /tmp/relay.py ros2-mavros:/tmp/relay.py
docker exec ros2-mavros bash -lc "source /opt/ros/*/setup.bash; timeout 40 python3 /tmp/ds.py"
docker exec ros2-mavros bash -lc "pkill -f relay.py || true"
docker exec -d ros2-mavros bash -lc "source /opt/ros/*/setup.bash; exec python3 /tmp/relay.py >/tmp/relay.log 2>&1"
sleep 5; docker exec ros2-mavros bash -lc "pgrep -f relay.py >/dev/null && echo relay OK || echo relay FAILED"'
