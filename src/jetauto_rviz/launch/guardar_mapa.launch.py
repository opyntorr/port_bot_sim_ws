#!/usr/bin/env python3
"""
guardar_mapa.launch.py — Guarda el mapa del SLAM DESDE LA LAPTOP (sin SSH manual).

Corre el nodo guardar_mapa (jetauto_rviz) en la laptop: se suscribe a /map por el bridge
DDS, escribe el .pgm/.yaml en ~/jetauto_maps (con timestamp si no das nombre) y ADEMAS lo
sube al Orin (~/maps) por scp para que navegar_real.launch.py pueda servirlo.

Requiere que el SLAM este corriendo (view.launch.py o mapear_real.launch.py).

Uso (terminal de la laptop):
    ros2 launch jetauto_rviz guardar_mapa.launch.py
    ros2 launch jetauto_rviz guardar_mapa.launch.py map_name:=frente208

Args:
  map_name:=<nombre>     nombre sin extension (default: mapa_<YYYYMMDD_HHMMSS>)
  output_dir:=<ruta>     carpeta local (default ~/jetauto_maps)
  orin_host:=jetson@10.42.1.1   destino scp ('' = no subir al Orin)
  orin_dir:=/home/jetson/maps   carpeta destino en el Orin
  cyclonedds_uri:=~/cyclonedds-laptop.xml
  ros_domain_id:=0
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_dds = os.path.expanduser('~/cyclonedds-laptop.xml')

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='',
                              description='Nombre del mapa (default: mapa_<timestamp>).'),
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser('~/jetauto_maps'),
                              description='Carpeta local en la laptop.'),
        DeclareLaunchArgument('orin_host', default_value='jetson@10.42.1.1',
                              description="Destino scp ('' = no subir al Orin)."),
        DeclareLaunchArgument('orin_dir', default_value='/home/jetson/maps',
                              description='Carpeta destino en el Orin.'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('cyclonedds_uri', default_value=default_dds),
        DeclareLaunchArgument('ros_domain_id', default_value='0'),

        # entorno DDS del bridge (para que /map del Orin llegue a este nodo local)
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', ['file://', LaunchConfiguration('cyclonedds_uri')]),

        Node(
            package='jetauto_rviz', executable='guardar_mapa', name='guardar_mapa_laptop',
            output='screen',
            parameters=[{
                'map_name': LaunchConfiguration('map_name'),
                'output_dir': LaunchConfiguration('output_dir'),
                'orin_host': LaunchConfiguration('orin_host'),
                'orin_dir': LaunchConfiguration('orin_dir'),
                'map_topic': LaunchConfiguration('map_topic'),
            }],
        ),
    ])
