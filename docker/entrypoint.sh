#!/bin/bash
set -e

# Source do ROS2
source /opt/ros/foxy/setup.bash

# Source do workspace local (se existir)
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Exportar variáveis de ambiente necessárias
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Executar comando passado
exec "$@"