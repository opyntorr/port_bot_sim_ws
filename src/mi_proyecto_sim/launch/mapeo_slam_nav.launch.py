"""
mapeo_slam_nav.launch.py — Construir un mapa con SLAM Toolbox en SIM, manejando el JetAuto
manualmente con un mando Xbox. (Para RE-MAPEAR el laberinto tras cambios al mundo/modelo.)

Incluye:
  - Gazebo con el mundo del laberinto.
  - JetAuto via jetauto_bringup (RSP + spawn + controladores + jetauto_chassis_sim +
    odometria nativa de gz -> /odom + TF odom->base_footprint + puente de sensores).
  - filtro_lidar (/scan -> /scan_filtered, FOV 190, anti auto-oclusion) que consume SLAM.
  - Joy + teleop_twist_joy (mando Xbox) -> /cmd_vel.
  - SLAM Toolbox en modo mapping (online_async) con config/mapper_params_online_async.yaml.
    Arranca recien cuando el odom publica odom->base_footprint (compuerta wait_for_tf); si
    arranca antes, SLAM descarta los primeros scans ("queue is full"). Un TimerAction de
    reloj de pared no sirve con RTF bajo, por eso se espera la TF en sim-time.
  - RViz con rviz/mi_config.rviz.

NO carga mapa previo ni stack de navegacion: es SOLO para mapear. Para navegar con el mapa
resultante usa navegacion_slam.launch.py.

Para guardar el mapa, en otra terminal mientras la sim corre:
    ros2 run mi_proyecto_sim guardar_mapa_slam.py --ros-args \\
        -p output_dir:=src/mi_proyecto_sim/maps \\
        -p map_name:=mapa_laberinto
(Guardalo como 'mapa_laberinto' para que navegacion_slam lo tome por default.)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable,
    IncludeLaunchDescription, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco_pid.urdf.xacro')
    slam_params = os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml')
    xbox_params = os.path.join(pkg_sim, 'config', 'xbox_mecanum.yaml')

    # =========================================================
    # GAZEBO
    # =========================================================
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_dir + ':' + pkg_sim,  # pkg_sim: mallas del JetAuto
    )
    plugin_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib',
    )
    gui_fov_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_GUI_CAMERA_FOV',
        value='0.691',
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
    )

    # Puente: solo /clock. Sensores del robot (/scan, /cam_1/*, /imu/data_raw) y odom/TF
    # los aporta jetauto_bringup. /cmd_vel lo consume jetauto_chassis_sim (lado ROS).
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    # =========================================================
    # ROBOT JETAUTO (bringup reutilizable)
    # =========================================================
    jetauto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'jetauto_bringup.launch.py')
        ),
        launch_arguments={
            'x': '-1.35', 'y': '-1.35', 'z': '0.08', 'yaw': '1.5708',
            'use_sim_time': 'true',
        }.items(),
    )

    # =========================================================
    # CONFIGURACION COMUN (SLAM, RViz, Teleop)
    # =========================================================
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
        parameters=[ParameterFile(xbox_params, allow_substs=True)],
        remappings=[
            ('joy', '/joy'),
            ('cmd_vel', '/cmd_vel'),
        ],
        output='screen',
    )


    # Compuerta: NO arrancar SLAM hasta que el odom publique odom->base_footprint. Si arranca
    # antes, SLAM tira los primeros scans ("queue is full"). Un TimerAction (reloj de pared) no
    # sirve con RTF bajo; esto espera la TF en sim-time.
    espera_odom = Node(
        package='mi_proyecto_sim',
        executable='wait_for_tf.py',
        name='espera_odom_tf',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target': 'odom',
            'source': 'base_footprint',
            'timeout': 60.0,
        }],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    slam_tras_odom = RegisterEventHandler(
        OnProcessExit(target_action=espera_odom, on_exit=[slam_toolbox_launch])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'mi_config.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================
    # LANZAR TODO (robot via include jetauto + SLAM + teleop + RViz)
    # =========================================================
    return LaunchDescription([
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        jetauto,
        joy_node,
        teleop,
        espera_odom,        # espera odom->base_footprint y entonces dispara SLAM
        slam_tras_odom,
        rviz_node,
    ])
