import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nodo de control de trayectoria puro para el hardware real (sin spawner de Gazebo)
    control_trayectoria_node = Node(
        package='mi_proyecto_sim',
        executable='control_trayectoria.py',
        name='control_trayectoria',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        control_trayectoria_node,
    ])
