#!/usr/bin/env python3
"""
GLUE LAUNCH — correr el cerebro AGV de mi_proyecto_sim sobre el JetAuto REAL (Jetson Orin).

Compone la capa de robot real (nuestro port ROS2 del JetAuto: base I2C + lidar + cámara + SLAM)
con el cerebro de navegación de mi_proyecto_sim (RRT + Kelly&Díaz + ArUco). El cerebro es
agnóstico del robot: solo consume /odom, /scan, /cmd_vel, /cam_1/image y el árbol TF
map->odom->base_footprint->lidar_frame, que esta capa provee.

CONTRATO (verificado): el ÚNICO remap necesario es la cámara -> /cam_1/image.
/odom, /scan, /cmd_vel, /map y los frames ya coinciden con nuestro stack.

DOS MODOS (arg `dron`):
  dron:=false  (DEFAULT, lidar-SLAM + mapa pregrabado)  -> sin el Tello:
      * map_server sirve un mapa guardado del laberinto como /map_dron (mapa global del planner)
      * publicador_tfs_arucos lee un arucos.yaml estático -> publica /alignment_ready (desbloquea control)
      * slam_toolbox da /map (obstáculos dinámicos) + TF map->odom
  dron:=true   (misión completa con Tello)  -> mision_dron genera /map_dron + arucos.yaml en vivo,
      detector_aruco hace el GPS-visual. (Requiere el dron + 2º WiFi USB en la Orin — ver P3.)

USO (en la Orin, con el JetAuto cableado):
  ros2 launch mi_proyecto_sim mision_jetauto.launch.py
  # opciones: dron:=true | start_base:=false (dry-run con rosbag) | map_dron_yaml:=... | arucos_yaml:=...

NOTA: este launch es la v1 de integración; el comportamiento exacto de /map_dron vs /map y el
gating de alineación se afina en la validación en sim (Fase 5 del plan) antes del hardware.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('mi_proyecto_sim')
    bringup_share = get_package_share_directory('jetauto_bringup')
    slam_share = get_package_share_directory('jetauto_slam')

    default_map = os.path.join(sim_share, 'maps', 'mapa_laberinto.yaml')
    default_arucos = os.path.join(sim_share, 'maps', 'arucos.yaml')

    # ---- argumentos ----
    start_base = LaunchConfiguration('start_base')   # base+lidar real (false = dry-run con rosbag)
    use_slam = LaunchConfiguration('use_slam')
    use_camera = LaunchConfiguration('use_camera')
    dron = LaunchConfiguration('dron')               # false = modo mapa-pregrabado (sin Tello)
    video_device = LaunchConfiguration('video_device')
    map_dron_yaml = LaunchConfiguration('map_dron_yaml')
    arucos_yaml = LaunchConfiguration('arucos_yaml')

    args = [
        DeclareLaunchArgument('start_base', default_value='true',
                              description='Lanzar la base real (jetauto_bringup/robot.launch.py). false para dry-run con rosbag.'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('dron', default_value='false',
                              description='true = misión con Tello (genera /map_dron + arucos.yaml en vivo). false = mapa pregrabado.'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Cámara RGB UVC del Astra Pro (para el ArUco de la etapa final).'),
        DeclareLaunchArgument('map_dron_yaml', default_value=default_map,
                              description='Mapa guardado del laberinto servido como /map_dron (modo sin dron).'),
        DeclareLaunchArgument('arucos_yaml', default_value=default_arucos,
                              description='arucos.yaml estático para publicador_tfs_arucos (modo sin dron).'),
    ]

    # ---- capa de ROBOT REAL (nuestro port) ----
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'robot.launch.py')),
        condition=IfCondition(start_base))

    # Cámara RGB del carrito -> /cam_1/image (ÚNICO remap del contrato).
    # No necesita CameraInfo (el ArUco del carrito es detección 2D).
    camara = Node(
        package='v4l2_camera', executable='v4l2_camera_node',
        name='cam_1', output='screen',
        parameters=[{
            'video_device': video_device,
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
        }],
        remappings=[
            ('image_raw', '/cam_1/image'),
            ('camera_info', '/cam_1/camera_info'),
        ],
        condition=IfCondition(use_camera))

    # SLAM (slam_toolbox) -> /map (obstáculos dinámicos) + TF map->odom.
    # start_robot:=false porque la base ya la lanza robot.launch.py arriba.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'slam.launch.py')),
        launch_arguments={'start_robot': 'false'}.items(),
        condition=IfCondition(use_slam))

    # ---- CEREBRO (mi_proyecto_sim) ----
    filtro_lidar = Node(
        package='mi_proyecto_sim', executable='filtro_lidar.py',
        name='filtro_lidar', output='screen')

    planificador = Node(
        package='mi_proyecto_sim', executable='planificador_rrt',
        name='planificador_rrt', output='screen')

    control = Node(
        package='mi_proyecto_sim', executable='control_trayectoria.py',
        name='control_trayectoria', output='screen',
        parameters=[{'use_sim_time': False}])

    # ---- MODO SIN DRON (mapa pregrabado): /map_dron desde map_server + arucos.yaml estático ----
    # map_server publica el mapa guardado; lo remapeamos /map -> /map_dron (el planner lo espera ahí,
    # latched/transient_local). SLAM sigue dueño de /map. Lifecycle manager lo activa solo.
    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server_dron', output='screen',
        parameters=[{'yaml_filename': map_dron_yaml, 'use_sim_time': False, 'frame_id': 'map'}],
        remappings=[('/map', '/map_dron')],
        condition=UnlessCondition(dron))

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map_dron', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': ['map_server_dron']}],
        condition=UnlessCondition(dron))

    # publicador_tfs_arucos lee arucos.yaml -> publica /alignment_ready (desbloquea control_trayectoria)
    # + TFs map->map_dron_origin->aruco_N. En modo sin dron usamos el arucos.yaml estático.
    publicador_arucos = Node(
        package='mi_proyecto_sim', executable='publicador_tfs_arucos.py',
        name='publicador_tfs_arucos', output='screen',
        parameters=[{'arucos_yaml': arucos_yaml}])

    # ---- MODO DRON (Tello): GPS-visual + generación de mapa en vivo (P3, validación posterior) ----
    # Requiere el Tello + 2º adaptador WiFi USB en la Orin. Se valida cuando esté el hardware.
    detector_aruco = Node(
        package='mi_proyecto_sim', executable='detector_aruco.py',
        name='detector_aruco', output='screen',
        condition=IfCondition(dron))
    # mision_dron + driver Tello se añaden aquí cuando se valide el dron (ver mision_dron_real.launch.py).

    return LaunchDescription(args + [
        base,
        camara,
        slam,
        filtro_lidar,
        planificador,
        control,
        # modo sin dron:
        map_server,
        map_server_lifecycle,
        publicador_arucos,
        # modo dron:
        detector_aruco,
    ])
