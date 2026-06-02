"""
Launch para navegacion manual del carrito con goal puesto desde RViz.

Flujo:
  1. Carga un mapa previamente generado (PGM + YAML) en map_server.
  2. AMCL localiza al carrito en el mapa (frame `map`). Para fijar la pose
     inicial usa la tool "2D Pose Estimate" de RViz, que publica /initialpose.
  3. El usuario fija el destino con "2D Goal Pose" en RViz (-> /goal_pose).
  4. nav_goal_bridge convierte /goal_pose en una TF `meta_aruco` y publica
     la TF `carrito_aruco` (pose actual del robot). Tambien publica la TF
     `map -> map_dron_origin` (identidad) y /alignment_ready=True para
     desbloquear control_trayectoria.
  5. planificador_rrt planea una ruta entre `carrito_aruco` y `meta_aruco`
     usando el mapa cargado.
  6. control_trayectoria sigue la ruta. Modos de busqueda y estacionamiento
     visual estan deshabilitados (disable_visual_modes=True): al llegar al
     terminal_arrival el robot se detiene.

Argumentos:
  map  : path absoluto al .yaml del mapa (default: mapa_mision.yaml en
         src/mi_proyecto_sim/maps)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable,
    TimerAction, RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    default_map = os.path.join(maps_dir, 'mapa_mision.yaml')

    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco_pid.urdf.xacro')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path al YAML del mapa a cargar',
    )
    map_yaml = LaunchConfiguration('map')

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

    # Puente: solo /clock. Sensores del robot (/scan, /cam_1/*, /imu/data_raw) y
    # odom/TF los aporta jetauto_bringup. /cmd_vel lo consume jetauto_chassis_sim (ROS).
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
    # MAP SERVER PLANNER
    # /map_dron -> para planificador_rrt (mismo yaml, distinto frame_id/topic)
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
    # SLAM TOOLBOX (Mapping)
    # =========================================================
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    # =========================================================
    # NAV GOAL BRIDGE
    # Publica meta_aruco (desde /goal_pose), carrito_aruco (pose actual),
    # TF map -> map_dron_origin (identidad) y /alignment_ready=True.
    # =========================================================
    nav_goal_bridge = Node(
        package='mi_proyecto_sim',
        executable='nav_goal_bridge.py',
        name='nav_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================
    # PLANIFICADOR + CONTROL DE TRAYECTORIA
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

    control_trayectoria_node = Node(
        package='mi_proyecto_sim',
        executable='control_trayectoria.py',
        name='control_trayectoria',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'disable_visual_modes': True,
        }],
    )

    # =========================================================
    # RVIZ
    # =========================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'mi_config.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    # Damos margen al TF tree y al map_server antes de arrancar SLAM y planner.
    delayed_nav_stack = TimerAction(
        period=5.0,
        actions=[
            slam_toolbox_launch,
            lifecycle_manager,
            nav_goal_bridge,
            planificador_rrt_node,
            control_trayectoria_node,
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
        rviz_node,
        delayed_nav_stack,
    ])
