#!/usr/bin/env python3
"""
navegar_real.launch.py — UN comando para NAVEGAR el JetAuto real (cargar mapa + goal en RViz).

Arranca, todo desde la laptop:
  1. RViz (config view.rviz) para ver el mapa y mandar el destino con "2D Goal Pose".
  2. El cerebro de LOCALIZACION en el ORIN por SSH (orin_brain_start.sh
     localizacion_nav_bridge.launch.py map:=... = map_server(/map_dron) + wait_for_tf +
     slam_toolbox + lifecycle + nav_goal_bridge + planificador_rrt + control_diferencial).
     Se DETIENE al cerrar este launch (OnShutdown).
  3. Teleop OPCIONAL (teleop:=true) como respaldo manual.

Flujo: arranca -> en RViz pulsa "2D Goal Pose" -> nav_goal_bridge -> RRT -> control_diferencial
-> /cmd_vel -> el robot va a la meta.

Precondiciones (siempre arriba): contenedor del Nano + jetauto-orin.service (RSP+EKF).
NO usar view.launch.py slam:=true en paralelo (doble slam -> conflicto TF).

Correr desde una terminal del ESCRITORIO (RViz necesita DISPLAY):
    ros2 launch jetauto_rviz navegar_real.launch.py
    ros2 launch jetauto_rviz navegar_real.launch.py map:=/home/jetson/maps/mi_mapa.yaml

Args:
  map:=/home/jetson/maps/mapa_real.yaml   mapa previo en /map_dron (ruta EN EL ORIN, para el RRT)
  teleop:=false|true               teleop de respaldo (default false)
  rviz:=true|false                 arrancar RViz (default true)
  orin_host:=jetson@10.42.1.1
  cyclonedds_uri:=~/cyclonedds-laptop.xml
  ros_domain_id:=0
  joy_dev:=/dev/input/js0
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
    teleop_cfg = os.path.join(get_package_share_directory('jetauto_teleop'), 'config', 'teleop.yaml')
    default_dds = os.path.expanduser('~/cyclonedds-laptop.xml')

    orin = LaunchConfiguration('orin_host')

    # Cerebro de LOCALIZACION en el Orin (SSH), pasando el mapa. El script es idempotente.
    start_brain = ExecuteProcess(
        cmd=['ssh', '-tt'] + SSH_OPTS
            + ['-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
               orin, ['bash ', LaunchConfiguration('orin_brain_script'),
                      ' localizacion_nav_bridge.launch.py map:=', LaunchConfiguration('map')]],
        name='orin_brain_nav', output='screen')

    # Limpieza determinista al cerrar: mata todo el brain de localizacion en el Orin.
    stop_brain = ExecuteProcess(
        cmd=['ssh'] + SSH_OPTS
            + [orin, 'pkill -9 -f localizacion_nav_bridge; pkill -9 -f async_slam_toolbox; '
                     'pkill -9 -f control_diferencial; pkill -9 -f planificador_rrt; '
                     'pkill -9 -f nav_goal_bridge; pkill -9 -f map_server; '
                     'pkill -9 -f filtro_lidar; pkill -9 -f wait_for_tf; true'],
        name='orin_brain_stop', output='screen')

    return LaunchDescription([
        # El map: se pasa al SSH y se resuelve en el ORIN -> usar su home (/home/jetson), NO el de la laptop.
        DeclareLaunchArgument('map', default_value='/home/jetson/maps/mapa_real.yaml',
                              description='YAML del mapa previo a cargar en /map_dron (ruta EN EL ORIN).'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('orin_host', default_value='jetson@10.42.1.1'),
        DeclareLaunchArgument('orin_brain_script', default_value='/home/jetson/orin_brain_start.sh'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        # Teleop de respaldo (opcional) — local
        Node(package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
             parameters=[{'dev': LaunchConfiguration('joy_dev'),
                          'deadzone': 0.08, 'autorepeat_rate': 20.0}],
             output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node', parameters=[teleop_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),

        # RViz — local (manda el goal con 2D Goal Pose)
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),

        # Cerebro de localizacion en el Orin (SSH) + limpieza al cerrar
        start_brain,
        RegisterEventHandler(OnShutdown(on_shutdown=[stop_brain])),
    ])
