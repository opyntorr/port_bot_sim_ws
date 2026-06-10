"""
solo_carrito_vacio.launch.py  —  Banco de depuracion del carrito en un mundo VACIO.

Copia LEAN de solo_carrito.launch.py para aislar el comportamiento del lidar / la marcha
sin las paredes del laberinto. Spawnea SOLO:
  - mundo vacio (worlds/vacio.sdf: piso + luz, sin modelos)
  - robot JetAuto (jetauto_bringup: RSP + spawn + controladores + chassis + IMU + EKF + sensores)
  - filtro_lidar  (/scan -> /scan_filtered con self_clearance)
  - RViz + joy + teleop  (para conducir manualmente con el control y ver /scan_filtered)

NO incluye SLAM / map_server / publicador_tfs / planificador_rrt / control_trayectoria:
en un mundo sin paredes SLAM no tiene features para localizar y el control no recibiria ruta.

Uso tipico (auto-oclusion):
  ros2 launch mi_proyecto_sim solo_carrito_vacio.launch.py
  # En RViz: Fixed Frame = odom (o base_footprint). Mira /scan_filtered: en espacio abierto
  # NO deberia haber puntos pegados al robot. Si los hay -> el robot se ve a si mismo.
  # Para medir la distancia frontal cruda directamente:
  #   ros2 topic echo /scan_filtered --once   (o visual en RViz)
  # Conduce hacia adelante con el control para confirmar que +X avanza fisicamente.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    world_file = os.path.join(pkg_sim, 'worlds', 'vacio.sdf')
    models_dir = os.path.join(pkg_sim, 'models')

    # Variables de entorno para que Gazebo encuentre los modelos y mallas del JetAuto.
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_dir + ':' + pkg_sim,
    )
    plugin_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib',
    )
    gui_fov_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_GUI_CAMERA_FOV',
        value='0.691',
    )

    # Gazebo con el mundo vacio.
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
    )

    # Puente ROS <-> Gazebo: solo /clock (los sensores los puentea jetauto_bringup).
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    # Robot JetAuto (bringup reutilizable).
    jetauto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'jetauto_bringup.launch.py')
        ),
        launch_arguments={
            'x': '0.0', 'y': '0.0', 'z': '0.08', 'yaw': '0.0',
            'use_sim_time': 'true',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'mi_config.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            ParameterFile(
                os.path.join(pkg_sim, 'config', 'xbox_mecanum.yaml'),
                allow_substs=True,
            )
        ],
        remappings=[
            ('joy', '/joy'),
            ('cmd_vel', '/cmd_vel'),
        ],
        output='screen',
    )


    return LaunchDescription([
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        jetauto,
        rviz_node,
        joy_node,
        teleop,
    ])
