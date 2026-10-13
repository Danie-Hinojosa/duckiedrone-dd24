#!/bin/bash
# dd-tof-fix: workaround for the dt-ros2-interface bug (image 2026-09-23) where the config
# from px4_config.yaml and the /mavros/bottom_tof remap never reach the MAVROS plugins,
# so the bottom ToF never reaches PX4 (no altitude -> failsafe ends in TERMINATE).
# Every 10 s: if the relay is not running inside ros2-mavros (new or restarted container),
# copy the scripts, load the px4_config.yaml parameters into the MAVROS plugins and start the relay.
DIR=/opt/dd-tof-fix
C=ros2-mavros
log() { echo "dd-tof-fix: $*"; }
while true; do
  if docker inspect -f '{{.State.Running}}' $C 2>/dev/null | grep -q true; then
    if ! docker exec $C pgrep -f /tmp/relay.py >/dev/null 2>&1; then
      log "relay not running in $C; applying workaround"
      docker cp $DIR/ds.py $C:/tmp/ds.py && docker cp $DIR/relay.py $C:/tmp/relay.py
      if docker exec $C bash -lc "source /opt/ros/*/setup.bash; timeout 40 python3 /tmp/ds.py"; then
        docker exec -d $C bash -lc "source /opt/ros/*/setup.bash; exec python3 /tmp/relay.py >/tmp/relay.log 2>&1"
        log "workaround applied"
      else
        log "mavros not ready yet; retrying"
      fi
    fi
  fi
  sleep 10
done
