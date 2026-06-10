"""
solo_carrito_obstaculos_lidar.launch.py

Clon de solo_carrito.launch.py + obstaculos variables + control_trayectoria, pero con
SLAM basado COMPLETAMENTE en el LiDAR: la odometria del carrito (la nativa de gz) se APAGA
y la odometria odom->base_footprint la genera rf2o_laser_odometry a partir del /scan.

Diferencias vs solo_carrito.launch.py:
  1. jetauto_bringup se incluye con publish_sim_odom_tf:=false -> el simulador NO publica
     /odom ni la TF odom->base_footprint.
  2. rf2o_laser_odometry consume /scan_filtered y publica:
       - TF odom->base_footprint  (la que antes daba gz; ahora 100% LiDAR)
       - /odom  (Odometry; lo consume control_trayectoria.py)
     Asi slam_toolbox (que usa la TF odom->base como prior + compuerta de scans) trabaja
     sobre una odometria derivada solo del LiDAR, sin tocar la odometria del carrito.
  3. Se spawnea el mesh de obstaculos variables (obstaculos_var) en Gazebo y se lanza
     control_trayectoria.py (igual que control_con_obstaculos.launch.py).

La compuerta espera_ekf (espera la TF odom->base_footprint) sigue sirviendo: ahora esa TF la
da rf2o, asi que SLAM no arranca hasta que rf2o publique la primera odometria.

Requisitos previos (artefactos de una corrida anterior de la mision del dron):
  - src/mi_proyecto_sim/maps/mapa_mision.pgm / .yaml
  - src/mi_proyecto_sim/maps/arucos.yaml

Requiere el paquete rf2o_laser_odometry compilado en el workspace.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable,
    TimerAction, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    arucos_yaml = os.path.join(maps_dir, 'arucos.yaml')
    mapa_yaml = os.path.join(maps_dir, 'mapa_mision.yaml')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    obstaculos_sdf = os.path.join(pkg_sim, 'models', 'obstaculos_var', 'model.sdf')

    # 1. Entorno para que Gazebo encuentre los modelos (pkg_sim: mallas del JetAuto).
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

    # 2. Gazebo (con GUI).
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
    )

    # 3. Puente ROS <-> Gazebo: solo /clock. Sensores y odom del robot -> jetauto_bringup.
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    visor_carrito = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view_carrito',
        arguments=['/cam_1/image_aruco'],
        output='screen',
    )

    # =========================================================
    # ROBOT JETAUTO — con la odometria nativa de gz APAGADA (publish_sim_odom_tf:=false).
    # La TF odom->base_footprint la dara rf2o (abajo), no el simulador.
    # =========================================================
    jetauto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'jetauto_bringup.launch.py')
        ),
        launch_arguments={
            'x': '-1.0', 'y': '-1.0', 'z': '0.08', 'yaw': '1.5708',
            'use_sim_time': 'true',
            'publish_sim_odom_tf': 'false',   # <-- odom SOLO-LiDAR
        }.items(),
    )


    # =========================================================
    # ODOMETRIA SOLO-LiDAR: rf2o_laser_odometry.
    #   - consume /scan_filtered (sin auto-oclusion del lidar)
    #   - publica TF odom->base_footprint  (la que SLAM usa de prior)
    #   - publica /odom  (Odometry; lo consume control_trayectoria.py)
    #   init_pose_from_topic="" -> arranca en el origen sin esperar pose externa.
    # =========================================================
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'laser_scan_topic': '/scan_filtered',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
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

    # =========================================================
    # MAPA PREVIO (mision dron) — /map_dron (frame map_dron_origin) para el planner.
    # publicador_tfs_arucos conecta map_dron_origin con `map` (SLAM) via TF estatica.
    # =========================================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': mapa_yaml,
            'use_sim_time': True,
            'frame_id': 'map_dron_origin',
        }],
        remappings=[('/map', '/map_dron')],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    publicador_tfs_node = Node(
        package='mi_proyecto_sim',
        executable='publicador_tfs_arucos.py',
        name='publicador_tfs_arucos',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'arucos_yaml': arucos_yaml},
        ],
    )

    # SLAM Toolbox (mapping) — mapa nuevo en vivo /map + TF map->odom. scan_topic=/scan_filtered.
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

    # Compuerta: NO arrancar SLAM hasta tener la TF odom->base_footprint (ahora la da rf2o).
    espera_odom = Node(
        package='mi_proyecto_sim', executable='wait_for_tf.py', name='espera_odom_tf',
        output='screen',
        parameters=[{'use_sim_time': True, 'target': 'odom', 'source': 'base_footprint',
                     'timeout': 60.0}],
    )
    slam_tras_odom = RegisterEventHandler(
        OnProcessExit(target_action=espera_odom, on_exit=[slam_toolbox_launch])
    )

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

    # =========================================================
    # OBSTACULOS VARIABLES + CONTROL DE TRAYECTORIA (de control_con_obstaculos.launch.py)
    #  - El mesh se spawnea en Gazebo (misma pose que el modelo `laberinto_real`).
    #  - Diferido para que Gazebo ya este arriba antes del spawn.
    # =========================================================
    spawn_obstaculos = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_obstaculos_var',
        arguments=[
            '-name', 'obstaculos_var',
            '-file', obstaculos_sdf,
            '-x', '-2.294',
            '-y', '2.294',
            '-z', '0.0',
            '-R', '1.5708',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )
    spawn_obstaculos_diferido = TimerAction(period=6.0, actions=[spawn_obstaculos])

    control_trayectoria_node = Node(
        package='mi_proyecto_sim',
        executable='control_trayectoria.py',
        name='control_trayectoria',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    # Tras spawnear los obstaculos, arrancar el control (margen extra para la fisica del mesh).
    arrancar_control = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_obstaculos,
            on_exit=[TimerAction(period=1.5, actions=[control_trayectoria_node])],
        )
    )

    return LaunchDescription([
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        jetauto,
        rf2o_node,             # odometria solo-LiDAR (odom->base_footprint + /odom)
        visor_carrito,
        rviz_node,
        joy_node,
        teleop,
        map_server_node,
        lifecycle_manager_node,
        publicador_tfs_node,
        espera_odom,           # espera odom->base_footprint (de rf2o) y dispara SLAM
        slam_tras_odom,
        planificador_rrt_node,
        spawn_obstaculos_diferido,
        arrancar_control,
    ])
