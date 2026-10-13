#!/bin/bash
# Installs the dd-tof-fix service on the drone. Usage: ~/duckiedrone/workaround/install.sh
# Uninstall (on the drone): sudo systemctl disable --now dd-tof-fix && sudo rm -rf /opt/dd-tof-fix /etc/systemd/system/dd-tof-fix.service
set -e
H=${1:-duckie@duckiedrone01.local}
D=$(dirname "$(readlink -f "$0")")
scp -q "$D/ds.py" "$D/relay.py" "$D/dd-tof-fix.sh" "$D/dd-tof-fix.service" $H:/tmp/
ssh -t $H 'sudo mkdir -p /opt/dd-tof-fix && sudo install -m 644 /tmp/ds.py /tmp/relay.py /opt/dd-tof-fix/ && sudo install -m 755 /tmp/dd-tof-fix.sh /opt/dd-tof-fix/ && sudo install -m 644 /tmp/dd-tof-fix.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now dd-tof-fix && sleep 15 && journalctl -u dd-tof-fix -n 5 --no-pager'
