#!/bin/bash
# Arranca slam_toolbox en el Orin para el bridge (/scan del Nano + /odom del EKF).
# Lo invoca el view.launch.py de la laptop por SSH. IDEMPOTENTE: mata cualquier slam
# previo (evita duplicados/leftovers si una sesion anterior quedo colgada por WiFi).
pkill -9 -f slam.launch.py    2>/dev/null
pkill -9 -f async_slam_toolbox 2>/dev/null
sleep 1
source /opt/ros/humble/setup.bash
source /home/jetson/jetauto_ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml
exec ros2 launch jetauto_slam slam.launch.py start_robot:=false use_rviz:=false
