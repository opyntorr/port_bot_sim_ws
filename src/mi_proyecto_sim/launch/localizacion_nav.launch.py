"""
localizacion_nav.launch.py  —  Navegacion del JetAuto con AMCL + meta desde RViz.

RECONSTRUIDO desde el bytecode (.pyc) de la version original (que se habia perdido: el
.py quedo en 0 bytes en todas las copias del workspace; el unico registro era el
__pycache__/localizacion_nav.launch.cpython-310.pyc de 4 KB). La version original usaba
el rosmaster viejo (carrito_con_aruco.urdf.xacro, /cmd_vel->gz, odom de /model/rosmaster_x3/*);
aqui esta ADAPTADO al JetAuto (jetauto_bringup) y al patron actual del proyecto.

Flujo:
  1. Gazebo (laberinto, CON GUI) + JetAuto (jetauto_bringup: RSP + spawn + controladores +
     chassis + IMU + EKF odom->base_footprint + sensores) + filtro_lidar (/scan -> /scan_filtered).
  2. map_server_planner carga el MAPA PREVIO (mapa_mision) y lo publica en /map_dron
     (frame map_dron_origin). Es solo referencia para el planner; NO se sobrescribe.
  3. SLAM toolbox (mode=mapping, scan_topic=/scan_filtered) reconstruye un MAPA NUEVO en vivo
     y lo publica en /map (frame `map`), ademas de la TF `map -> odom` (localizacion). Asi hay
     dos mapas independientes: el cargado (/map_dron) y el que SLAM va construyendo (/map). SLAM
     arranca recien cuando el EKF publica odom->base_footprint (compuerta wait_for_tf), si no
     descarta los primeros scans. Para guardar el mapa nuevo usa guardar_mapa_slam.py con OTRO
     nombre de archivo (no mapa_mision) para no pisar el previo.
  4. Fijas el destino con "2D Goal Pose" en RViz (-> /goal_pose). nav_goal_bridge lo
     convierte en la TF `meta_aruco`, publica `carrito_aruco` (pose actual) y `map ->
     map_dron_origin` (identidad) + /alignment_ready=True para destrabar el control.
  5. planificador_rrt planea (mapa en /map_dron, frame map_dron_origin) y control_diferencial
     sigue la ruta.
  6. joy + teleop_twist_joy: puedes manejar con control Xbox en cualquier momento.

Diferencia vs navegacion_manual.launch.py: comparten SLAM toolbox como localizacion, pero aqui
ademas se carga un mapa previo en /map_dron, se trae nav_goal_bridge + planificador_rrt +
control_diferencial y el teleop Xbox.

Argumentos:
  map : path absoluto al .yaml del MAPA PREVIO a cargar en /map_dron (default: mapa_mision.yaml
        en src/mi_proyecto_sim/maps). SLAM construye su propio mapa aparte (/map), no este.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable,
    TimerAction, DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    default_map = os.path.join(maps_dir, 'mapa_mision.yaml')

    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path al YAML del mapa a cargar (AMCL + planner)',
    )
    map_yaml = LaunchConfiguration('map')

    # =========================================================
    # GAZEBO + ENTORNO
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
        cmd=['ign', 'gazebo', '-r', world_file],  # con GUI (sin -s)
        output='screen',
    )

    # Puente: solo /clock. Sensores/odom/TF del robot los aporta jetauto_bringup;
    # /cmd_vel lo consume jetauto_chassis_sim (lado ROS).
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
            'x': '-1.0', 'y': '-1.0', 'z': '0.08', 'yaw': '1.5708',
            'use_sim_time': 'true',
        }.items(),
    )

    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================
    # MAPA PREVIO (solo /map_dron, frame map_dron_origin) -> para planificador_rrt.
    # NO se publica /map aqui: ese topic y la TF `map` los produce SLAM toolbox (mapa nuevo
    # en vivo). nav_goal_bridge publica map -> map_dron_origin (identidad) para conectar
    # ambos frames. Asi el mapa cargado (/map_dron) y el de SLAM (/map) conviven sin pisarse.
    # =========================================================
    map_server_planner = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_planner',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': True,
            'frame_id': 'map_dron_origin',
        }],
        remappings=[('/map', '/map_dron')],
    )

    # =========================================================
    # SLAM TOOLBOX (localizacion + mapa NUEVO en vivo) — publica /map y la TF map->odom.
    # Reemplaza a AMCL. Arranca recien cuando el EKF publica odom->base_footprint (compuerta
    # wait_for_tf); si arranca antes, SLAM descarta los primeros scans ("queue is full").
    # =========================================================
    espera_ekf = Node(
        package='mi_proyecto_sim',
        executable='wait_for_tf.py',
        name='espera_ekf_tf',
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
            'slam_params_file': os.path.join(
                pkg_sim, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    slam_tras_ekf = RegisterEventHandler(
        OnProcessExit(target_action=espera_ekf, on_exit=[slam_toolbox_launch])
    )

    # lifecycle_manager: solo gestiona el map_server del MAPA PREVIO (/map_dron).
    # SLAM toolbox maneja su propio ciclo de vida; no va aqui.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server_planner'],
        }],
    )

    # =========================================================
    # NAV GOAL BRIDGE (/goal_pose -> meta_aruco; carrito_aruco; identidad; /alignment_ready)
    # =========================================================
    nav_goal_bridge = Node(
        package='mi_proyecto_sim',
        executable='nav_goal_bridge.py',
        name='nav_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================
    # PLANIFICADOR + CONTROL
    # =========================================================
    planificador_rrt_node = Node(
        package='mi_proyecto_sim',
        executable='planificador_rrt',
        name='planificador_rrt',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_radius_m': 0.19,
        }],
    )

    control_diferencial_node = Node(
        package='mi_proyecto_sim',
        executable='control_diferencial.py',
        name='control_diferencial',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    # =========================================================
    # TELEOP XBOX + RVIZ
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'mi_config.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    # Margen para que el TF tree (jetauto_bringup) y el map_server esten arriba antes
    # de arrancar lifecycle, nav_goal_bridge, planner y control. SLAM NO va aqui: lo
    # dispara el event handler cuando el EKF publica odom->base_footprint.
    delayed_nav_stack = TimerAction(
        period=5.0,
        actions=[
            lifecycle_manager,
            nav_goal_bridge,
            planificador_rrt_node,
            control_diferencial_node,
        ],
    )

    return LaunchDescription([
        map_arg,
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        jetauto,
        filtro_lidar_node,
        map_server_planner,
        joy_node,
        teleop,
        rviz_node,
        espera_ekf,        # espera odom->base_footprint y entonces dispara SLAM
        slam_tras_ekf,
        delayed_nav_stack,
    ])
