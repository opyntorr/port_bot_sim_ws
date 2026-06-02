#!/usr/bin/env python3
"""
mapear_real.launch.py — UN comando para MAPEAR el JetAuto real con el control.

Arranca, todo desde la laptop:
  1. Teleop (joy_linux + teleop_twist_joy -> /cmd_vel) para manejar con el control Xbox/8BitDo.
  2. RViz (config view.rviz) para ver el mapa/scan/TF.
  3. El cerebro de MAPEO en el ORIN por SSH (orin_brain_start.sh mapeo_slam_nav_bridge.launch.py
     = filtro_lidar + slam_toolbox mapping). Se DETIENE al cerrar este launch (OnShutdown).

Precondiciones (siempre arriba): contenedor del Nano + jetauto-orin.service (RSP+EKF).
NO usa el SLAM de view.launch.py: el cerebro de mapeo trae su PROPIO slam_toolbox.

Correr desde una terminal del ESCRITORIO (RViz necesita DISPLAY):
    ros2 launch jetauto_rviz mapear_real.launch.py

Para guardar el mapa (en otra terminal, SSH al Orin):
    ros2 run mi_proyecto_sim guardar_mapa_slam.py --ros-args -p output_dir:=$HOME/maps -p map_name:=mapa_real

Args:
  teleop:=true|false     arrancar el teleop del control (default true)
  rviz:=true|false       arrancar RViz (default true)
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

    # Cerebro de MAPEO en el Orin (SSH). El script es idempotente (mata cualquier brain previo).
    start_brain = ExecuteProcess(
        cmd=['ssh', '-tt'] + SSH_OPTS
            + ['-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2',
               orin, ['bash ', LaunchConfiguration('orin_brain_script'),
                      ' mapeo_slam_nav_bridge.launch.py']],
        name='orin_brain_mapeo', output='screen')

    # Limpieza determinista al cerrar: mata el brain de mapeo en el Orin.
    stop_brain = ExecuteProcess(
        cmd=['ssh'] + SSH_OPTS
            + [orin, 'pkill -9 -f mapeo_slam_nav_bridge; pkill -9 -f async_slam_toolbox; '
                     'pkill -9 -f filtro_lidar; true'],
        name='orin_brain_stop', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('orin_host', default_value='jetson@10.42.1.1'),
        DeclareLaunchArgument('orin_brain_script', default_value='/home/jetson/orin_brain_start.sh'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),

        # entorno DDS del bridge para los nodos LOCALES (teleop, rviz)
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        # Teleop (control) — nodos locales en la laptop
        Node(package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
             parameters=[{'dev': LaunchConfiguration('joy_dev'),
                          'deadzone': 0.08, 'autorepeat_rate': 20.0}],
             output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node', parameters=[teleop_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),

        # RViz — local
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),

        # Cerebro de mapeo en el Orin (SSH) + limpieza al cerrar
        start_brain,
        RegisterEventHandler(OnShutdown(on_shutdown=[stop_brain])),
    ])
