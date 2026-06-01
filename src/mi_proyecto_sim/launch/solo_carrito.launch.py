"""
Launch para probar SOLO el carrito reutilizando el mapa y los ArUcos
generados por una corrida previa de la mision del dron.

Requisitos previos (artefactos de una corrida anterior):
  - src/mi_proyecto_sim/maps/mapa_mision.pgm
  - src/mi_proyecto_sim/maps/mapa_mision.yaml
  - src/mi_proyecto_sim/maps/arucos.yaml

Diferencias vs simulacion.launch.py:
  - No se spawnea el dron ni se corre su mision.
  - publicador_tfs_arucos, slam_occupancy_grid y planificador_rrt se lanzan
    directamente al inicio (sin esperar a que termine el dron).
  - Se omiten los nodos de control del dron (optitrack_sim, pose_fuser,
    position_controller, plotter) y los visores/bridges asociados.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable, TimerAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    mapa_pgm = os.path.join(maps_dir, 'mapa_mision.pgm')
    arucos_yaml = os.path.join(maps_dir, 'arucos.yaml')
    mapa_yaml = os.path.join(maps_dir, 'mapa_mision.yaml')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco_pid.urdf.xacro')

    # 1. Variables de entorno para que Gazebo encuentre los modelos
    #    pkg_sim incluido para hallar las mallas del JetAuto (urdf/jetauto + meshes/jetauto).
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

    # 2. Lanzar Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
    )

    # 3. Puente ROS <-> Gazebo. Solo /clock; los sensores del robot (/scan, /cam_1/*,
    #    /imu/data_raw) los puentea jetauto_bringup.launch.py. /cmd_vel NO se puentea a gz:
    #    lo consume jetauto_chassis_sim en el lado ROS.
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
    # ROBOT JETAUTO (bringup reutilizable: RSP + spawn + controladores +
    # chassis_sim + IMU madgwick + EKF + puente de sensores)
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

    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================
    # ARTEFACTOS DE LA CORRIDA ANTERIOR (MISION DRON)
    # =========================================================

    # Servidor de Mapas (Nav2) para publicar el mapa estático del dron (stitching).
    # frame_id='map_dron_origin' lo conecta a `map` (SLAM) via TF estatica
    # publicada por publicador_tfs_arucos.
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
        remappings=[('/map', '/map_dron')]  # Publicar en /map_dron para visualización pura
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # Republica las TFs estaticas de los ArUcos leyendo arucos.yaml previo.
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

    # SLAM Toolbox oficial - usa online_async_launch.py con yaml dedicado
    # (patron tomado de dinav2/warehouse_bot, mantiene scan_topic=/scan_filtered).
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

    # Compuerta: NO arrancar SLAM hasta que el EKF publique odom->base_footprint. Si arranca
    # antes, SLAM tira los primeros scans ("queue is full") porque no puede ubicarlos sin esa
    # TF. Un TimerAction (reloj de pared) no sirve con RTF bajo; esto espera la TF en sim-time.
    espera_ekf = Node(
        package='mi_proyecto_sim', executable='wait_for_tf.py', name='espera_ekf_tf',
        output='screen',
        parameters=[{'use_sim_time': True, 'target': 'odom', 'source': 'base_footprint',
                     'timeout': 60.0}],
    )
    slam_tras_ekf = RegisterEventHandler(
        OnProcessExit(target_action=espera_ekf, on_exit=[slam_toolbox_launch])
    )

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

    # =========================================================
    # PUENTES PARA LA CONSOLA WEB
    # =========================================================
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'use_sim_time': True, 'port': 9090}],
        output='screen',
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'use_sim_time': True,
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10_000_000,
        }],
        output='screen',
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080}],
        output='screen',
    )

    return LaunchDescription([
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        jetauto,
        visor_carrito,
        rviz_node,
        joy_node,
        teleop,
        filtro_lidar_node,
        map_server_node,
        lifecycle_manager_node,
        publicador_tfs_node,
        espera_ekf,            # espera odom->base_footprint y entonces dispara SLAM
        slam_tras_ekf,
        planificador_rrt_node,
        # Consola web (paquetes no instalados en el contenedor actual).
        # Si instalas rosbridge_server / foxglove_bridge / web_video_server,
        # vuelve a añadirlos aqui.
        # rosbridge_server,
        # foxglove_bridge_node,
        # web_video_server_node,
    ])
