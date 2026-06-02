#!/usr/bin/env python3
"""
Teleop del JetAuto con control Xbox/8BitDo desde la laptop.

Arranca joy_linux_node (lee el joystick -> /joy) + teleop_twist_joy (/joy -> /cmd_vel)
y fija el entorno DDS del bridge para sus nodos, asi /cmd_vel llega al chassis del Nano.

Uso:  ros2 launch jetauto_teleop teleop.launch.py
Args (portabilidad a otra PC):
  joy_dev:=/dev/input/js0          dispositivo del control
  cyclonedds_uri:=~/cyclonedds-laptop.xml   XML de CycloneDDS (peers del bridge)
  ros_domain_id:=0                 dominio DDS del bridge
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('jetauto_teleop'), 'config', 'teleop.yaml')
    default_dds = os.path.expanduser('~/cyclonedds-laptop.xml')

    return LaunchDescription([
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='Dispositivo del control (joystick).'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds,
                              description='Ruta absoluta al XML de CycloneDDS de la laptop.'),
        DeclareLaunchArgument('ros_domain_id', default_value='0',
                              description='ROS_DOMAIN_ID del bridge JetAuto.'),

        # entorno DDS del bridge (solo para los nodos de este launch)
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        # joy_linux (API legacy /dev/input/js0; el joy_node de SDL no entrega eventos headless)
        Node(package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
             parameters=[{
                 'dev': LaunchConfiguration('joy_dev'),
                 'deadzone': 0.08,
                 'autorepeat_rate': 20.0,
             }],
             output='screen'),

        # teleop_twist_joy: /joy -> /cmd_vel (holonomico; LB hombre-muerto, RB turbo)
        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop_twist_joy_node',
             parameters=[cfg],
             output='screen'),
    ])
