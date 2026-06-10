#!/usr/bin/env python3
"""
Abre RViz en la LAPTOP para comparar el EKF manual vs la odometria, con el env DDS
del bridge JetAuto ya fijado (dominio 0 + cyclonedds-laptop.xml).

Uso:
    ros2 launch ~/agv_uav_project_jetauto/ekf_manual/ekf_view.launch.py

Muestra: /odometry/ekf_manual (rojo), /odom (azul), /scan (blanco), TF, /map.
Fixed Frame = map -> necesita tu localizacion (slam_toolbox) corriendo para que el
marco 'map' exista; si aun no, cambia el Fixed Frame a 'odom' en RViz.
"""
import os
import sys
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node

HOME = os.path.expanduser('~')
HERE = os.path.dirname(os.path.abspath(__file__))
RVIZ_CFG = os.path.join(HERE, 'ekf_compare.rviz')
EKF_NODE = os.path.join(HERE, 'ekf_manual_node.py')
CYCLONE = os.path.join(HOME, 'cyclonedds-laptop.xml')


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', 'file://' + CYCLONE),
        # NOTA: el nodo EKF corre en el ORIN (IMU local y limpio), no aqui — la IMU
        # best_effort sobre WiFi es fragil. Arrancalo en el Orin con run_ekf.sh.
        # Este launch es solo el visor (RViz) en la laptop.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_ekf',
            arguments=['-d', RVIZ_CFG],
            output='screen',
        ),
    ])
