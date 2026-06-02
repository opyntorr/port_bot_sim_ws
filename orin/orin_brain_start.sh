#!/bin/bash
# Arranca un launch del "cerebro" mi_proyecto_sim en el Orin para el bridge JetAuto.
# Lo invoca la laptop por SSH (estilo orin_slam_start.sh). Fija el entorno DDS del bridge.
#
# Uso:
#   bash ~/orin_brain_start.sh mapeo_slam_nav_bridge.launch.py
#   bash ~/orin_brain_start.sh localizacion_nav_bridge.launch.py map:=/home/jetson/maps/mapa_real.yaml
#
# IDEMPOTENTE: mata cualquier brain/slam previo para no duplicar nodos.
# OJO: NO usar pkill con el nombre del .launch.py — este bash recibe ese nombre como
# argumento, asi que `pkill -f <launch>` mataria su PROPIO shell (exit 255 antes del exec).
# Se mata el `ros2 launch` previo por un patron que NO aparece en el argv de este bash,
# y los nodos por su ejecutable real.
pkill -9 -f 'ros2 launch mi_proyecto_sim' 2>/dev/null
pkill -9 -f async_slam_toolbox  2>/dev/null
pkill -9 -f control_diferencial 2>/dev/null
pkill -9 -f planificador_rrt    2>/dev/null
pkill -9 -f nav_goal_bridge     2>/dev/null
pkill -9 -f map_server_planner  2>/dev/null
pkill -9 -f map_server          2>/dev/null
pkill -9 -f filtro_lidar        2>/dev/null
pkill -9 -f wait_for_tf         2>/dev/null
sleep 1

export HOME=/home/jetson
source /opt/ros/humble/setup.bash
source /home/jetson/jetauto_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml

exec ros2 launch mi_proyecto_sim "$@"
