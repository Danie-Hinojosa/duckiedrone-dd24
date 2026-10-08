#!/bin/bash
# dd-tof-fix: workaround para el bug de dt-ros2-interface (img 2026-09-23) donde la config
# de px4_config.yaml y el remap /mavros/bottom_tof no llegan a los plugins de MAVROS,
# por lo que el ToF inferior nunca llega a PX4 (sin altitud -> failsafe termina en TERMINATE).
# Cada 10 s: si el relay no corre dentro de ros2-mavros (contenedor nuevo/reiniciado),
# copia los scripts, carga en los plugins de MAVROS los parámetros de px4_config.yaml y arranca el relay.
DIR=/opt/dd-tof-fix
C=ros2-mavros
log() { echo "dd-tof-fix: $*"; }
while true; do
  if docker inspect -f '{{.State.Running}}' $C 2>/dev/null | grep -q true; then
    if ! docker exec $C pgrep -f /tmp/relay.py >/dev/null 2>&1; then
      log "relay no está corriendo en $C; aplicando workaround"
      docker cp $DIR/ds.py $C:/tmp/ds.py && docker cp $DIR/relay.py $C:/tmp/relay.py
      if docker exec $C bash -lc "source /opt/ros/*/setup.bash; timeout 40 python3 /tmp/ds.py"; then
        docker exec -d $C bash -lc "source /opt/ros/*/setup.bash; exec python3 /tmp/relay.py >/tmp/relay.log 2>&1"
        log "workaround aplicado"
      else
        log "mavros aún no listo; reintento"
      fi
    fi
  fi
  sleep 10
done
