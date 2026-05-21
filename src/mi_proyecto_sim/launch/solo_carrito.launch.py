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
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable
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
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco.urdf.xacro')

    # 1. Variables de entorno para que Gazebo encuentre los modelos
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_dir,
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

    # 3. Puente ROS <-> Gazebo (solo lo que usa el carrito)
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/rosmaster_x3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/rosmaster_x3/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/rosmaster_x3/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
        ],
        remappings=[
            ('/model/rosmaster_x3/odometry', '/odom_raw'),
            ('/model/rosmaster_x3/tf', '/tf_raw'),
            ('/model/rosmaster_x3/pose', '/odom_pose'),
        ],
        output='screen',
    )

    visor_carrito = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view_carrito',
        arguments=['/cam_1/image'],
        output='screen',
    )

    # =========================================================
    # CARRITO ROSMASTER
    # =========================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' use_gazebo:=true']),
                value_type=str,
            ),
        }],
    )

    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rosmaster_x3',
            '-topic', 'robot_description',
            '-x', '-1.0',
            '-y', '-1.0',
            '-z', '0.1',
            '-Y', '1.5708',
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_ws/mi_config.rviz'],
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
    # ARTEFACTOS DE LA CORRIDA ANTERIOR
    # =========================================================

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

    # Usa el PGM previo como prior del SLAM.
    slam_occupancy_grid_node = Node(
        package='mi_proyecto_sim',
        executable='slam_occupancy_grid.py',
        name='slam_occupancy_grid',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pgm_path': mapa_pgm,
            'pgm_resolution': 0.002,
            'pgm_origin_x': -3.35,
            'pgm_origin_y': -3.84,
            'map_resolution': 0.05,
            'map_width': 160,
            'map_height': 180,
            'map_origin_x': -4.0,
            'map_origin_y': -4.5,
            'map_to_odom_yaw': 0.0,
            # AMCL se encarga de publicar map->odom; este nodo solo da el mapa.
            'publish_map_to_odom_tf': False,
        }],
    )

    # AMCL casero: localizacion por filtro de particulas usando /map + /scan_filtered.
    # Publica TF map->odom corrigiendo continuamente la deriva de la odom de ruedas.
    amcl_node = Node(
        package='mi_proyecto_sim',
        executable='amcl_localizer.py',
        name='amcl_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'num_particles': 500,
            'laser_max_range': 5.0,
            'sigma_hit': 0.35,  # Más permisivo con desalineaciones del mapa (antes 0.2)
            'z_hit': 0.90,
            'z_rand': 0.10,     # Mayor tolerancia a obstáculos dinámicos/no mapeados
            'laser_subsample': 10,
            'update_min_d': 0.10,
            'update_min_a': 0.10,
            'init_from_aruco': True,
            'init_std_xy': 0.3,
            'init_std_yaw': 0.3,
        }],
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

    odom_noise_node = Node(
        package='mi_proyecto_sim',
        executable='odom_noise_filter.py',
        name='odom_noise_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'noise_std_x': 0.002,         # 2 mm de ruido aleatorio por segundo
            'noise_std_y': 0.002,         # 2 mm de ruido lateral
            'noise_std_yaw': 0.001,       # Ligera vibración rotacional
            'drift_x_per_sec': 0.001,     # Deriva de 1 mm por segundo (ej. llantas desgastadas)
            'drift_y_per_sec': -0.0005,   # Ligera deriva lateral
            'drift_yaw_per_sec': 0.0005,  # Deriva de ~1.7 grados por minuto (error de calibración de llantas)
        }],
    )

    return LaunchDescription([
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        visor_carrito,
        robot_state_publisher,
        spawner,
        rviz_node,
        joy_node,
        teleop,
        filtro_lidar_node,
        publicador_tfs_node,
        slam_occupancy_grid_node,
        amcl_node,
        planificador_rrt_node,
        rosbridge_server,
        foxglove_bridge_node,
        web_video_server_node,
        odom_noise_node,
    ])
