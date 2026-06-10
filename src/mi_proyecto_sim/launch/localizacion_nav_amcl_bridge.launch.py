"""
localizacion_nav_amcl_bridge.launch.py  —  Navegacion del JetAuto REAL con AMCL (bridge Orin/Nano).

COPIA de localizacion_nav_bridge.launch.py pero la localizacion la hace **AMCL** (nav2) contra un
mapa de ocupacion FIJO (el .pgm/.yaml que ya tienes), en vez de slam_toolbox reconstruyendo un
mapa nuevo. AMCL = filtro de particulas que a cada hipotesis de pose le "castea" el scan del lidar
contra el grid (modelo likelihood_field) -> publica la TF `map -> odom`. NO vuelve a mapear.

Diferencias vs el launch de slam:
  - SIN slam_toolbox. En su lugar: map_server_amcl (publica el mapa fijo en /map, frame `map`) + amcl.
  - El mapa se carga DOS veces del mismo .yaml: /map (para AMCL) y /map_dron (referencia del planner,
    frame map_dron_origin). nav_goal_bridge sigue uniendo map -> map_dron_origin (identidad).
  - Pose inicial: AMCL arranca SIN pose; dale "2D Pose Estimate" en RViz (-> /initialpose). En el
    pipeline final, el aruco publica /initialpose. Hasta que no le des pose, no hay map->odom.

El hardware (LiDAR/ruedas/IMU) lo da el contenedor del Nano y el computo (RSP + EKF -> odom->base)
lo da jetauto-orin.service, ambos ya corriendo. El EKF (cmd_vel + IMU) sigue dando el odom; AMCL
solo corrige map->odom. Reloj REAL (chrony): use_sim_time = False.

Uso (en el Orin, con el entorno DDS del bridge ya cargado):
    ros2 launch mi_proyecto_sim localizacion_nav_amcl_bridge.launch.py map:=$HOME/maps/mi_mapa.yaml

Argumentos:
  map : path absoluto al .yaml del mapa FIJO a usar (AMCL + planner). default ~/maps/mapa_real.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    TimerAction, DeclareLaunchArgument, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = os.path.expanduser('~/maps/mapa_real.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path al YAML del mapa fijo (lo usan AMCL en /map y el planner en /map_dron)',
    )
    map_yaml = LaunchConfiguration('map')

    # /scan (BEST_EFFORT, del Nano) -> /scan_filtered (para la evasion del control/RRT).
    # AMCL en cambio usa /scan crudo (FOV completo) para localizar mejor.
    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # Mapa FIJO -> /map (frame `map`) para AMCL.
    map_server_amcl = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server_amcl',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False,
            'frame_id': 'map',
            'topic_name': 'map',
        }],
    )

    # Mapa FIJO -> /map_dron (frame map_dron_origin). Referencia del planner RRT.
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

    # AMCL: localizacion contra el grid fijo. Publica TF map->odom. Usa el odom del EKF como
    # modelo de movimiento y el /scan del Nano. Pose inicial via /initialpose (2D Pose Estimate).
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': 'scan',
            'map_topic': 'map',
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'laser_model_type': 'likelihood_field',
            'set_initial_pose': False,        # arranca sin pose -> dale 2D Pose Estimate / aruco
            'always_reset_initial_pose': False,
            'tf_broadcast': True,
            'transform_tolerance': 1.0,
            'min_particles': 500,
            'max_particles': 2000,
            'update_min_d': 0.15,             # m antes de actualizar (lidar tapado: actualiza seguido)
            'update_min_a': 0.15,             # rad antes de actualizar
            'resample_interval': 1,
            'laser_max_range': 12.0,
            'laser_min_range': 0.15,
            'max_beams': 120,
            'laser_likelihood_max_dist': 2.0,
            'sigma_hit': 0.2,
            'z_hit': 0.5,
            'z_rand': 0.5,
            'z_max': 0.05,
            'z_short': 0.05,
            # EVO ronda 1 (Grupo C): alpha1-4 0.2->0.3 -> mas ruido de movimiento para que las
            # particulas se abran y AMCL corrija mejor la odom dead-reckoning que driftea.
            'alpha1': 0.3, 'alpha2': 0.3, 'alpha3': 0.3, 'alpha4': 0.3, 'alpha5': 0.2,
            'do_beamskip': False,
            'pf_err': 0.05,
            'pf_z': 0.99,
            'recovery_alpha_slow': 0.0,
            'recovery_alpha_fast': 0.0,
            'save_pose_rate': 0.5,
        }],
    )

    # Compuerta: espera odom->base_footprint (del EKF de orin_compute) antes de activar el stack.
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

    # lifecycle_manager: gestiona los map_server + AMCL. Orden: el /map (amcl) primero, luego amcl,
    # luego el /map_dron del planner.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server_amcl', 'amcl', 'map_server_planner'],
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
            'robot_radius_m': 0.22,
        }],
    )

    control_diferencial_node = Node(
        package='mi_proyecto_sim',
        executable='control_diferencial.py',
        name='control_diferencial',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scan_topic': '/scan_filtered',  # robot real: conserva filtro_lidar (self_clearance)
        }],
    )

    # Cuando el EKF ya publica odom->base: activa lifecycle (map_servers + amcl) y arranca el
    # stack de navegacion. amcl/map_servers ya estan spawneados (esperando ser activados).
    nav_tras_ekf = RegisterEventHandler(
        OnProcessExit(target_action=espera_ekf, on_exit=[
            lifecycle_manager,
            nav_goal_bridge,
            planificador_rrt_node,
            control_diferencial_node,
        ])
    )

    return LaunchDescription([
        map_arg,
        filtro_lidar_node,
        map_server_amcl,
        map_server_planner,
        amcl_node,
        espera_ekf,        # espera odom->base y entonces activa el stack
        nav_tras_ekf,
    ])
