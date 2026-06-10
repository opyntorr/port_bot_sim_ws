"""
navegacion_slam.launch.py  —  Navegar el JetAuto en SIM (Gazebo) con SLAM (re-mapeando),
                              dando "2D Pose Estimate" + "2D Goal Pose" en RViz.

Equivalente en SIMULACION del cerebro del carrito REAL (navegar_real.launch.py ->
localizacion_nav_bridge.launch.py): MISMO pipeline de control, solo cambia el origen del
hardware (aqui Gazebo en vez del Nano/Orin). NO usa AMCL: SLAM toolbox (mode=mapping)
RECONSTRUYE un mapa nuevo en vivo y localiza (TF map->odom).

Flujo de RViz (los dos clicks que pediste):
  1. "2D Pose Estimate" (/initialpose): NO es para AMCL. nav_goal_bridge lo usa para ALINEAR
     el mapa de referencia precargado (/map_dron, frame map_dron_origin) con el mapa vivo de
     SLAM (/map). Hasta que no lo des, map->map_dron_origin es identidad; al darlo, congela la
     alineacion (tx,ty,phi) en la pose real del robot dentro del mapa de referencia.
  2. "2D Goal Pose" (/goal_pose): destino. nav_goal_bridge lo convierte en la TF meta_aruco,
     publica carrito_aruco (pose viva del robot) y /alignment_ready=True para destrabar el control.
  3. planificador_rrt planea (rejilla = /map_dron en map_dron_origin, fusionando el /map de SLAM
     para obstaculos dinamicos) y control_diferencial sigue la ruta -> /cmd_vel.

Componentes:
  - Gazebo (laberinto) + jetauto_bringup (RSP + spawn + controladores + chassis + odom gz
    odom->base_footprint + sensores) + filtro_lidar (/scan -> /scan_filtered, FOV 190, anti
    auto-oclusion).
  - map_server_planner: MAPA PREVIO -> /map_dron (frame map_dron_origin). Solo referencia del
    planner; NO se sobrescribe. El /map y la TF `map` los produce SLAM (mapa nuevo en vivo).
  - SLAM toolbox (mode=mapping, scan_topic=/scan_filtered). Arranca recien cuando el odom
    publica odom->base_footprint (compuerta wait_for_tf); si arranca antes descarta scans.
  - nav_goal_bridge + planificador_rrt + control_diferencial (igual que el carrito real).
  - RViz (mi_config.rviz: Fixed Frame=map, tools 2D Pose Estimate->/initialpose y 2D Goal
    Pose->/goal_pose). Teleop Xbox OPCIONAL de respaldo (teleop:=true).

Diferencia vs localizacion_nav.launch.py (que NO se toca): mismo pipeline SLAM, pero aqui el
teleop es opcional (default off) y el mapa default es mapa_laberinto.yaml (mapa_mision se borro).

Uso:
    ros2 launch mi_proyecto_sim navegacion_slam.launch.py
    ros2 launch mi_proyecto_sim navegacion_slam.launch.py map:=/ruta/a/mi_mapa.yaml
    ros2 launch mi_proyecto_sim navegacion_slam.launch.py teleop:=true

Argumentos:
  map    : path absoluto al .yaml del MAPA PREVIO -> /map_dron (referencia del planner RRT).
           default: mapa_laberinto.yaml en src/mi_proyecto_sim/maps. SLAM construye su propio
           mapa aparte (/map), no este.
  teleop : false|true  -> joy + teleop_twist_joy (mando Xbox) de respaldo. default false.
  rviz   : true|false  -> arrancar RViz. default true.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable,
    TimerAction, DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    default_map = os.path.join(maps_dir, 'mapa_laberinto.yaml')

    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path al YAML del MAPA PREVIO a cargar en /map_dron (referencia del planner).',
    )
    teleop_arg = DeclareLaunchArgument('teleop', default_value='false')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
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
            'x': '-1.35', 'y': '-1.35', 'z': '0.08', 'yaw': '1.5708',
            'use_sim_time': 'true',
        }.items(),
    )


    # =========================================================
    # MAPA PREVIO (solo /map_dron, frame map_dron_origin) -> rejilla del planificador_rrt.
    # NO se publica /map aqui: ese topic y la TF `map` los produce SLAM toolbox (mapa nuevo en
    # vivo). nav_goal_bridge publica map -> map_dron_origin para conectar ambos frames (identidad
    # hasta que el 2D Pose Estimate congela la alineacion real).
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
    # SLAM TOOLBOX (localizacion + mapa NUEVO en vivo) — publica /map y la TF map->odom.
    # Arranca recien cuando el odom publica odom->base_footprint (compuerta wait_for_tf); si
    # arranca antes, SLAM descarta los primeros scans ("queue is full").
    # =========================================================
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
            'slam_params_file': os.path.join(
                pkg_sim, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    slam_tras_odom = RegisterEventHandler(
        OnProcessExit(target_action=espera_odom, on_exit=[slam_toolbox_launch])
    )

    # =========================================================
    # NAV GOAL BRIDGE (/initialpose -> alinea map->map_dron_origin; /goal_pose -> meta_aruco;
    # carrito_aruco; /alignment_ready)
    # =========================================================
    nav_goal_bridge = Node(
        package='mi_proyecto_sim',
        executable='nav_goal_bridge.py',
        name='nav_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================
    # PLANIFICADOR + CONTROL (igual que el carrito real)
    # =========================================================
    planificador_rrt_node = Node(
        package='mi_proyecto_sim',
        executable='planificador_rrt',
        name='planificador_rrt',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_radius_m': 0.22,
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
    # TELEOP XBOX OPCIONAL (respaldo) + RVIZ
    # =========================================================
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop')),
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
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'mi_config.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Margen para que el TF tree (jetauto_bringup) y el map_server esten arriba antes de
    # arrancar lifecycle, nav_goal_bridge, planner y control. SLAM NO va aqui: lo dispara el
    # event handler cuando el odom publica odom->base_footprint.
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
        teleop_arg,
        rviz_arg,
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        jetauto,
        map_server_planner,
        joy_node,
        teleop,
        rviz_node,
        espera_odom,        # espera odom->base_footprint y entonces dispara SLAM
        slam_tras_odom,
        delayed_nav_stack,
    ])
