#!/bin/bash
# Reaplica el workaround del ToF->PX4 (bug de dt-ros2-interface). Se pierde al reiniciar ros2-mavros o el dron.
# Uso: ~/duckiedrone/workaround/apply.sh   (pide la contraseña de duckie si no hay sshpass/llave)
set -e
H=duckie@duckiedrone01.local
D=$(dirname "$0")
scp -q "$D/ds.py" "$D/relay.py" $H:/tmp/
ssh $H 'docker cp /tmp/ds.py ros2-mavros:/tmp/ds.py && docker cp /tmp/relay.py ros2-mavros:/tmp/relay.py
docker exec ros2-mavros bash -lc "source /opt/ros/*/setup.bash; timeout 40 python3 /tmp/ds.py"
docker exec ros2-mavros bash -lc "pkill -f relay.py || true"
docker exec -d ros2-mavros bash -lc "source /opt/ros/*/setup.bash; exec python3 /tmp/relay.py >/tmp/relay.log 2>&1"
sleep 5; docker exec ros2-mavros bash -lc "pgrep -f relay.py >/dev/null && echo relay OK || echo relay FAILED"'
