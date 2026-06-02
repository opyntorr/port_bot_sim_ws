#!/usr/bin/env python3
"""
RViz para monitorear el bridge JetAuto desde la laptop:
  TF + LaserScan (/scan, del Nano) + Map (/map, del slam_toolbox en el Orin).

1. Fija el entorno DDS del bridge para RViz (no hace falta sourcear nada aparte).
2. (slam:=true) Arranca slam_toolbox EN EL ORIN por SSH y lo DETIENE al cerrar
   (handler OnShutdown -> ssh pkill). El script orin_slam_start.sh es idempotente.

RViz hereda DISPLAY de la terminal -> correr desde una terminal del ESCRITORIO.

Uso:  ros2 launch jetauto_rviz view.launch.py
Args (portabilidad a otra PC):
  slam:=true|false                 arrancar/no el SLAM en el Orin
  orin_host:=jetson@10.42.1.1      usuario@IP del Orin (SSH)
  orin_slam_script:=/home/jetson/orin_slam_start.sh   script de SLAM en el Orin
  cyclonedds_uri:=~/cyclonedds-laptop.xml             XML de CycloneDDS
  ros_domain_id:=0                 dominio DDS del bridge
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            ExecuteProcess, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SSH_OPTS = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=no']


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('jetauto_rviz'), 'rviz', 'view.rviz')
    default_dds = os.path.expanduser('~/cyclonedds-laptop.xml')

    slam = LaunchConfiguration('slam')
    orin = LaunchConfiguration('orin_host')

    start_slam = ExecuteProcess(
        cmd=['ssh', '-tt'] + SSH_OPTS
            + ['-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
               orin, ['bash ', LaunchConfiguration('orin_slam_script')]],
        name='orin_slam', output='screen',
        condition=IfCondition(slam))

    # Limpieza determinista al cerrar RViz: SSH directo que mata el slam del Orin.
    stop_slam = ExecuteProcess(
        cmd=['ssh'] + SSH_OPTS
            + [orin, 'pkill -9 -f slam.launch.py; pkill -9 -f async_slam_toolbox; true'],
        name='orin_slam_stop', output='screen',
        condition=IfCondition(slam))

    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='true',
                              description='Arrancar slam_toolbox en el Orin por SSH (y detenerlo al cerrar).'),
        DeclareLaunchArgument('orin_host', default_value='jetson@10.42.1.1',
                              description='usuario@IP del Orin para SSH.'),
        DeclareLaunchArgument('orin_slam_script', default_value='/home/jetson/orin_slam_start.sh',
                              description='Ruta del script de arranque de SLAM en el Orin.'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds,
                              description='Ruta absoluta al XML de CycloneDDS de la laptop.'),
        DeclareLaunchArgument('ros_domain_id', default_value='0',
                              description='ROS_DOMAIN_ID del bridge JetAuto.'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        start_slam,
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen'),

        RegisterEventHandler(OnShutdown(on_shutdown=[stop_slam])),
    ])
