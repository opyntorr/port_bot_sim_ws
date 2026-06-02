#!/bin/bash
# orin_compute (RSP + madgwick + EKF) para el bridge.
# BLINDAJE DE ARRANQUE: al boot, enP8p1s0 (192.168.5.1) tarda en tener IP; si CycloneDDS
# se bindea antes -> "rmw_create_node: failed to create domain" y los nodos mueren (pero el
# launch queda vivo, asi que Restart=on-failure no reintentaba). Esperamos a la IP primero.
for i in $(seq 1 60); do
  ip -4 addr show enP8p1s0 2>/dev/null | grep -q "192.168.5.1" && break
  sleep 2
done
# margen extra para que el stack DDS/multicast quede listo
sleep 3

source /opt/ros/humble/setup.bash
source /home/jetson/jetauto_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
exec ros2 launch jetauto_bringup orin_compute.launch.py
