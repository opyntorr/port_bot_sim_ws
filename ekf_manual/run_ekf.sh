#!/usr/bin/env bash
# Arranca el EKF manual en el ORIN (dominio 0 del robot). No mueve el robot
# (solo publica /odometry/ekf_manual). Ctrl-C para salir.
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
DIR="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$DIR/ekf_manual_node.py" "$@"
