"""
localizacion_nav_bridge.launch.py  —  Navegacion del JetAuto REAL (bridge Orin/Nano).

Version "bridge" de localizacion_nav.launch.py SIN simulacion: NO lanza Gazebo, NI
ros_gz_bridge (/clock), NI el spawner jetauto_bringup, NI joy/teleop, NI RViz. El hardware
(LiDAR, ruedas, IMU) lo provee el contenedor del Nano y el computo (RSP + EKF -> /odom + TF
odom->base_footprint) lo provee jetauto-orin.service (orin_compute.launch.py), ambos ya
corriendo. El teleop (opcional) y RViz se lanzan APARTE desde la laptop.

Reloj REAL (chrony): use_sim_time = False en todos los nodos.

Flujo (identico a la sim, solo cambia el origen del hardware):
  1. map_server_planner carga el MAPA PREVIO y lo publica en /map_dron (frame map_dron_origin).
     Es solo referencia para el planner; NO se sobrescribe.
  2. SLAM Toolbox (mode=mapping, scan_topic=/scan_filtered) reconstruye un mapa NUEVO en vivo
     en /map (frame `map`) y publica la TF `map -> odom` (localizacion). Arranca recien cuando
     el EKF publica odom->base_footprint (compuerta wait_for_tf).
  3. Fijas el destino con "2D Goal Pose" en RViz (-> /goal_pose). nav_goal_bridge lo convierte
     en la TF `meta_aruco`, publica `carrito_aruco` (pose actual) y `map -> map_dron_origin`
     (identidad) + /alignment_ready=True para destrabar el control.
  4. planificador_rrt planea (mapa en /map_dron, frame map_dron_origin) y control_diferencial
     sigue la ruta publicando /cmd_vel (que el chassis del Nano consume).

Uso (en el Orin, con el entorno DDS del bridge ya cargado):
    ros2 launch mi_proyecto_sim localizacion_nav_bridge.launch.py \\
        map:=$HOME/maps/mapa_real.yaml

IMPORTANTE: NO lanzar a la vez `view.launch.py slam:=true` desde la laptop: arrancaria un
segundo slam_toolbox y habria conflicto de TF map->odom. Usar RViz con slam:=false.

Argumentos:
  map : path absoluto al .yaml del MAPA PREVIO a cargar en /map_dron
        (default: ~/maps/mapa_real.yaml). SLAM construye su propio mapa aparte (/map), no este.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    TimerAction, DeclareLaunchArgument, IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    # SLAM con el config VALIDADO en piso (jetauto_slam): scan_topic=/scan directo,
    # max_laser_range=12.0, min=0.15. NO el de sim (mi_proyecto_sim, max 5.0m) que degradaba
    # el mapeo/localizacion real. (filtro_lidar/scan_filtered SIGUE para el control/RRT, abajo.)
    slam_params = os.path.join(
        get_package_share_directory('jetauto_slam'),
        'config', 'mapper_params_online_async.yaml')
    default_map = os.path.expanduser('~/maps/mapa_real.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path al YAML del mapa previo a cargar en /map_dron (para el planner RRT)',
    )
    map_yaml = LaunchConfiguration('map')

    # /scan (BEST_EFFORT, del Nano) -> /scan_filtered (SLAM + evasion del control).
    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Mapa PREVIO -> /map_dron (frame map_dron_origin). Solo referencia para el planner.
    # /map y la TF `map` los produce SLAM toolbox (mapa nuevo en vivo); nav_goal_bridge
    # publica map -> map_dron_origin (identidad) para conectar ambos frames.
    map_server_planner = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_planner',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False,
            'frame_id': 'map_dron_origin',
        }],
        remappings=[('/map', '/map_dron')],
    )

    # Compuerta: espera odom->base_footprint (del EKF de orin_compute) antes de arrancar SLAM.
    espera_ekf = Node(
        package='mi_proyecto_sim',
        executable='wait_for_tf.py',
        name='espera_ekf_tf',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'target': 'odom',
            'source': 'base_footprint',
            'timeout': 60.0,
        }],
    )

    # SLAM Toolbox (localizacion + mapa nuevo en vivo). Publica /map y TF map->odom.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false',
        }.items(),
    )

    slam_tras_ekf = RegisterEventHandler(
        OnProcessExit(target_action=espera_ekf, on_exit=[slam_toolbox_launch])
    )

    # lifecycle_manager: solo gestiona el map_server del MAPA PREVIO (/map_dron).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server_planner'],
        }],
    )

    # /goal_pose (RViz) -> TF meta_aruco / carrito_aruco / map->map_dron_origin + /alignment_ready.
    nav_goal_bridge = Node(
        package='mi_proyecto_sim',
        executable='nav_goal_bridge.py',
        name='nav_goal_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    planificador_rrt_node = Node(
        package='mi_proyecto_sim',
        executable='planificador_rrt',
        name='planificador_rrt',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_radius_m': 0.19,
        }],
    )

    control_diferencial_node = Node(
        package='mi_proyecto_sim',
        executable='control_diferencial.py',
        name='control_diferencial',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    # Margen para que el map_server este arriba antes de lifecycle/nav_goal_bridge/planner/control.
    # SLAM NO va aqui: lo dispara el event handler cuando el EKF publica odom->base_footprint.
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
        filtro_lidar_node,
        map_server_planner,
        espera_ekf,        # espera odom->base_footprint y entonces dispara SLAM
        slam_tras_ekf,
        delayed_nav_stack,
    ])
