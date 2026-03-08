#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f "/root/colcon_ws/install/setup.bash" ]; then
    source /root/colcon_ws/install/setup.bash
fi

# golem_run için otomatik izin komutu
if [ -f "/root/colcon_ws/src/hexapod_controller/scripts/start_golem.sh" ]; then
    chmod +x /root/colcon_ws/src/hexapod_controller/scripts/start_golem.sh
fi

exec "$@"